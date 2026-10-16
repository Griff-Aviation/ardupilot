// Read-only DataFlash replay using the production detector. No MAVLink or actuator I/O.
#include "PayloadRelease.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

struct Field { char type; size_t offset; };
struct Format {
    unsigned length=0;
    std::string name;
    std::map<std::string,Field> fields;
};
static std::string text(const uint8_t *p,size_t length) {
    size_t n=0; while (n<length && p[n]) n++;
    return std::string(reinterpret_cast<const char*>(p),n);
}
static size_t size(char type) {
    switch (type) {
    case 'b': case 'B': case 'M': return 1;
    case 'h': case 'H': case 'c': case 'C': case 'g': return 2;
    case 'i': case 'I': case 'f': case 'e': case 'E': case 'L': case 'n': return 4;
    case 'q': case 'Q': case 'd': return 8;
    case 'N': return 16;
    case 'Z': case 'a': return 64;
    default: throw std::runtime_error("Unknown DataFlash field type");
    }
}
template<typename T> static double numeric(const uint8_t *p) {
    T value; std::memcpy(&value,p,sizeof(value)); return double(value);
}
static double value(const Format &format,const uint8_t *payload,const std::string &name,double fallback=0) {
    auto it=format.fields.find(name); if (it==format.fields.end()) return fallback;
    const Field &field=it->second; const uint8_t *p=payload+field.offset;
    switch(field.type) {
    case 'B': case 'M': return *p;
    case 'b': return numeric<int8_t>(p);
    case 'H': return numeric<uint16_t>(p);
    case 'h': return numeric<int16_t>(p);
    case 'I': return numeric<uint32_t>(p);
    case 'i': return numeric<int32_t>(p);
    case 'Q': return numeric<uint64_t>(p);
    case 'q': return numeric<int64_t>(p);
    case 'f': return numeric<float>(p);
    case 'd': return numeric<double>(p);
    case 'c': return numeric<int16_t>(p)*0.01;
    case 'C': return numeric<uint16_t>(p)*0.01;
    case 'e': return numeric<int32_t>(p)*0.01;
    case 'E': return numeric<uint32_t>(p)*0.01;
    case 'L': return numeric<int32_t>(p)*1.0e-7;
    default: return fallback;
    }
}
static std::string string_value(const Format &fmt,const uint8_t *p,const std::string &name) {
    auto it=fmt.fields.find(name); if (it==fmt.fields.end()) return {};
    return text(p+it->second.offset,size(it->second.type));
}
static double clamp(double x) { return std::max(-1.0,std::min(1.0,x)); }
static std::vector<double> thrust(double roll,double pitch,double yaw) {
    const double rad=3.14159265358979323846/180;
    roll*=rad; pitch*=rad; yaw*=rad;
    return {std::cos(yaw)*std::sin(pitch)*std::cos(roll)+std::sin(yaw)*std::sin(roll),
            std::sin(yaw)*std::sin(pitch)*std::cos(roll)-std::cos(yaw)*std::sin(roll),
            std::cos(pitch)*std::cos(roll)};
}
static std::string json_string(const std::string &s) {
    std::string out="\""; for (const char ch:s) { if(ch=='\\' || ch=='\"') out+='\\'; out+=ch; } return out+'\"';
}

int main(int argc,char **argv) try {
    if (argc<2 || argc>3) throw std::runtime_error("Usage: replay_payload_release LOG.BIN [--angle-max-only]");
    const bool angle_only=argc==3 && std::string(argv[2])=="--angle-max-only";
    std::ifstream file(argv[1],std::ios::binary|std::ios::ate);
    if (!file) throw std::runtime_error("Cannot open input log");
    const size_t bytes=size_t(file.tellg()); file.seekg(0);
    std::vector<uint8_t> data(bytes); file.read(reinterpret_cast<char*>(data.data()),bytes);
    if (size_t(file.gcount())!=bytes) throw std::runtime_error("Incomplete input read; download may still be active");
    Format formats[256]; formats[128].length=89; formats[128].name="FMT";
    std::map<std::string,double> params;
    std::map<std::string,unsigned> counts;
    PayloadRelease core; PayloadRelease::Input input;
    bool armed=false,landed=true,standby=false,have_attitude=false,have_pwm=false;
    bool saw_ang=false,arming_inferred=false;
    unsigned mode=0,armings=0,parse_errors=0,peak_pwm=0,updates=0;
    uint64_t first_time=0,last_time=0,last_att_time=0,last_pwm_time=0,max_att_gap=0,max_pwm_gap=0;
    double max_tilt=0,max_error=0;
    std::vector<std::string> triggers;
    auto param=[&](const std::string &name,double fallback) {
        const auto it=params.find(name); return it==params.end()?fallback:it->second;
    };
    auto arm=[&](bool on) {
        if (on==armed) return;
        armed=on;
        if (on) {
            PayloadRelease::Config cfg; cfg.enable=1; cfg.action=0;
            core.arm(cfg); armings++;
        } else { core.disarm(); landed=true; }
    };
    for (size_t pos=0;pos+3<=bytes;) {
        if (data[pos]!=0xA3 || data[pos+1]!=0x95) { pos++; parse_errors++; continue; }
        const unsigned type=data[pos+2];
        if (type==128) {
            if (pos+89>bytes) break;
            const uint8_t *p=data.data()+pos+3;
            Format fmt; fmt.length=p[1]; fmt.name=text(p+2,4);
            const std::string types=text(p+6,16),labels=text(p+22,64);
            std::istringstream columns(labels); std::string label; size_t offset=0;
            for (char ch:types) {
                if (!std::getline(columns,label,',')) throw std::runtime_error("Incomplete FMT labels");
                fmt.fields.emplace(label,Field{ch,offset}); offset+=size(ch);
            }
            if (offset+3!=fmt.length) throw std::runtime_error("Invalid FMT length: "+fmt.name);
            formats[p[0]]=fmt;
            pos+=89; continue;
        }
        const Format &fmt=formats[type];
        if (fmt.length<3) { pos++; parse_errors++; continue; }
        if (pos+fmt.length>bytes) break;
        const uint8_t *p=data.data()+pos+3; pos+=fmt.length;
        const std::string &name=fmt.name;
        if(first_time==0 && fmt.fields.count("TimeUS")) first_time=uint64_t(value(fmt,p,"TimeUS"));
        if (name!="PARM" && name!="ARM" && name!="EV" && name!="MODE" &&
            name!="ANG" && name!="ATT" && name!="RCOU") continue;
        counts[name]++;
        if (name=="PARM") {
            params[string_value(fmt,p,"Name")]=value(fmt,p,"Value"); continue;
        }
        uint64_t time=uint64_t(value(fmt,p,"TimeUS"));
        if (time<last_time) time=last_time; // file order is authoritative for held inputs
        if (name=="ARM") { arm(value(fmt,p,"ArmState")!=0); }
        if (name=="EV") {
            const int id=int(value(fmt,p,"Id"));
            if(id==10) arm(true);
            if(id==11) arm(false);
            if(id==18) landed=true;
            if(id==28) {
                // Some logs start after the arming event. NOT_LANDED is emitted
                // by the armed vehicle's land detector and establishes airborne state.
                if(!armed) { arm(true); arming_inferred=true; }
                landed=false;
            }
            if(id==74) standby=true;
            if(id==75) standby=false;
        }
        if (name=="MODE") mode=unsigned(value(fmt,p,"ModeNum",value(fmt,p,"Mode")));
        if (name=="ANG" || (name=="ATT" && !saw_ang)) {
            if(name=="ANG") saw_ang=true;
            const auto actual=thrust(value(fmt,p,"Roll"),value(fmt,p,"Pitch"),value(fmt,p,"Yaw"));
            const auto target=thrust(value(fmt,p,"DesRoll"),value(fmt,p,"DesPitch"),value(fmt,p,"DesYaw"));
            input.tilt_deg=float(std::acos(clamp(actual[2]))*180/3.14159265358979323846);
            input.error_deg=float(std::acos(clamp(actual[0]*target[0]+actual[1]*target[1]+actual[2]*target[2]))*180/3.14159265358979323846);
            have_attitude=true;
            if(last_att_time && armed) max_att_gap=std::max(max_att_gap,time-last_att_time);
            last_att_time=time;
            max_tilt=std::max(max_tilt,double(input.tilt_deg)); max_error=std::max(max_error,double(input.error_deg));
        }
        if(name=="RCOU") {
            input.enabled_motors=0;
            for(unsigned channel=1;channel<=32;channel++) {
                const unsigned function=unsigned(param("SERVO"+std::to_string(channel)+"_FUNCTION",0));
                int motor=-1;
                if(function>=33 && function<=40) motor=int(function-33);
                if(function>=82 && function<=85) motor=int(function-82+8);
                if(function>=160 && function<=179) motor=int(function-160+12);
                if(motor<0 || !fmt.fields.count("C"+std::to_string(channel))) continue;
                if(input.enabled_motors & (uint32_t(1)<<motor)) continue; // same first-channel policy as SRV_Channels
                const uint16_t pwm=uint16_t(value(fmt,p,"C"+std::to_string(channel)));
                input.enabled_motors|=uint32_t(1)<<motor; input.pwm[motor]=pwm;
                peak_pwm=std::max(peak_pwm,unsigned(pwm));
            }
            have_pwm=true;
            if(last_pwm_time && armed) max_pwm_gap=std::max(max_pwm_gap,time-last_pwm_time);
            last_pwm_time=time;
        }
        last_time=time;
        if(!armed || !have_attitude || !have_pwm) continue;
        const double angle=param("ANGLE_MAX",4500)*0.01;
        double limit=angle;
        if(mode==5 && !angle_only) {
            const double psc=param("PSC_ANGLE_MAX",0)>0?param("PSC_ANGLE_MAX",0):angle;
            const double loit=param("LOIT_ANG_MAX",0);
            limit=loit>0?std::min(loit,psc):std::min(angle,psc)*(2.0/3.0);
        }
        input.limit_deg=float(limit);
        input.flying=!landed;
        input.eligible=!standby && mode!=1 && mode!=14 && mode!=28;
        core.update(uint32_t(time/1000),input); updates++;
        const uint16_t events=core.take_events();
        if(events & PayloadRelease::Triggered) {
            std::ostringstream event;
            unsigned output=0;
            const unsigned trigger=unsigned(core.trigger_motor());
            for(unsigned ch=1;ch<=32;ch++) {
                const unsigned fn=unsigned(param("SERVO"+std::to_string(ch)+"_FUNCTION",0));
                if((trigger<=8 && fn==32+trigger) || (trigger>=9 && trigger<=12 && fn==73+trigger) ||
                   (trigger>=13 && fn==147+trigger)) { output=ch; break; }
            }
            event << "{\"time_s\":" << time*1.0e-6 << ",\"time_from_first_record_s\":" << (time-first_time)*1.0e-6
                  << ",\"motor\":" << trigger << ",\"output_channel\":" << output
                  << ",\"tilt_deg\":" << input.tilt_deg << ",\"limit_deg\":" << core.tilt_limit()
                  << ",\"error_deg\":" << input.error_deg << ",\"error_rate_deg_s\":" << core.error_rate()
                  << ",\"saturation_s\":" << core.saturation_ms()*0.001 << ",\"mode\":" << mode << "}";
            triggers.push_back(event.str());
        }
    }
    if(!have_attitude || !have_pwm || armings==0 || updates==0 || peak_pwm==0) {
        std::cerr << "attitude=" << have_attitude << " pwm=" << have_pwm << " armings=" << armings
                  << " updates=" << updates << " peak=" << peak_pwm << " resync=" << parse_errors
                  << " servo1=" << param("SERVO1_FUNCTION",-1) << " params=" << params.size() << '\n';
        for(const auto &count:counts) std::cerr << count.first << '=' << count.second << ' ';
        std::cerr << '\n';
        throw std::runtime_error("Missing attitude/PWM/arming/motor mapping data; cannot validate this log");
    }
    std::cout << "{\"file\":" << json_string(argv[1]) << ",\"bytes\":" << bytes
              << ",\"first_record_s\":" << first_time*1.0e-6
              << ",\"angle_max_only\":" << (angle_only?"true":"false")
              << ",\"arming_periods\":" << armings << ",\"updates\":" << updates
              << ",\"arming_inferred_from_not_landed\":" << (arming_inferred?"true":"false")
              << ",\"resync_bytes\":" << parse_errors << ",\"peak_motor_pwm\":" << peak_pwm
              << ",\"max_tilt_deg\":" << max_tilt << ",\"max_error_deg\":" << max_error
              << ",\"max_attitude_gap_s\":" << max_att_gap*1.0e-6
              << ",\"max_pwm_gap_s\":" << max_pwm_gap*1.0e-6
              << ",\"loit_ang_max\":" << param("LOIT_ANG_MAX",0)
              << ",\"angle_max_deg\":" << param("ANGLE_MAX",4500)*0.01
              << ",\"trigger_count\":" << triggers.size() << ",\"triggers\":[";
    for(size_t i=0;i<triggers.size();i++) { if(i) std::cout<<','; std::cout<<triggers[i]; }
    std::cout << "],\"records\":{";
    bool first=true;
    for(const auto &count:counts) { if(!first) std::cout<<','; first=false; std::cout<<json_string(count.first)<<':'<<count.second; }
    std::cout << "}}\n";
} catch(const std::exception &e) {
    std::cerr << "REPLAY_FAILED=" << e.what() << '\n'; return 1;
}
