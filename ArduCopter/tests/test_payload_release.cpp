// Standalone production-core tests; run Tools/scripts/test_payload_release.py.
#include "PayloadRelease.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>

static unsigned checks;
static void check(bool condition, const char *message)
{
    checks++;
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s (check %u)\n", message, checks);
        std::exit(1);
    }
}
using Core = PayloadRelease;
using State = Core::State;
static Core::Config config(bool release=false)
{
    Core::Config c;
    c.enable = 1;
    c.action = release ? 1 : 0;
    return c;
}
static Core::Input input()
{
    Core::Input in;
    in.flying = in.eligible = true;
    in.enabled_motors = 1;
    in.pwm[0] = 2000;
    in.limit_deg = 20;
    in.tilt_deg = 30;
    return in;
}
static void grow(Core &core, Core::Input &in, uint32_t start, unsigned duration)
{
    for (unsigned t=0; t<=duration; t+=10) {
        in.error_deg = 5 + t * 0.01f;
        core.update(start+t, in);
    }
}
static Core pending()
{
    Core c;
    c.arm(config(true));
    auto in=input();
    grow(c,in,0,500);
    check(c.state()==State::Pending,"release transaction starts at 500 ms");
    return c;
}

int main()
{
    {
        Core::Config c;
        check(c.valid() && c.enable==0 && c.action==0,"disabled/warn defaults");
        c.enable=1;
        check(c.valid(),"default enabled configuration valid");
        c.time_s=std::numeric_limits<float>::quiet_NaN();
        check(!c.valid(),"NaN time rejected");
        c=config(); c.angle_pct=std::numeric_limits<float>::infinity(); check(!c.valid(),"infinite factor rejected");
        c=config(); c.sysid=0; check(!c.valid(),"system broadcast rejected");
        c=config(); c.compid=256; check(!c.valid(),"component overflow rejected");
        c=config(); c.mode=1; check(!c.valid(),"unsupported mode rejected");
        c=config(); c.action=2; check(!c.valid(),"unsupported action rejected");
        c=config(); c.enable=2; check(!c.valid(),"invalid enable rejected");
        c=config(); c.time_s=0.099f; check(!c.valid(),"too short interval rejected");
        c=config(); c.pwm=2201; check(!c.valid(),"invalid PWM rejected");
        Core core; core.arm(c); auto in=input(); grow(core,in,0,1000);
        check(core.state()==State::InvalidConfig && !core.latched(),"invalid config cannot trigger");
    }
    for (unsigned scenario=0;scenario<7;scenario++) {
        Core c;
        c.arm(config());
        auto in=input();
        for (unsigned t=0;t<=3000;t+=10) {
            in.error_deg=t*0.01f;
            if (scenario==0) in.tilt_deg=10; // saturation + rising error, no tilt
            if (scenario==1) in.pwm[0]=1949; // tilt + error, no saturation
            if (scenario==2) in.error_deg=40; // tilt + saturation, no growth
            if (scenario==3) in.error_deg=50-t*0.01f; // decreasing
            if (scenario==4) in.eligible=false;
            if (scenario==5) in.flying=false;
            if (scenario==6) in.enabled_motors=0;
            c.update(t,in);
            check(!c.latched() && !c.wants_send(t),"incomplete conditions never trigger");
        }
    }
    {
        Core c; auto cfg=config(); c.arm(cfg); auto in=input();
        // Mutating the external configuration after arm must not enable release.
        cfg.action=1; cfg.pwm=1000;
        for (unsigned t=0;t<=600;t+=10) {
            in.error_deg=t*0.01f;
            c.update(t,in);
            check(c.latched()==(t>=500),"500 ms inclusive boundary");
            check(!c.wants_send(t),"warn mode never transmits");
        }
        check(c.state()==State::WarnLatched && c.trigger_motor()==1,"warn latch and motor attribution");
        in.eligible=false; c.update(700,in);
        check(c.latched(),"mode exit retains fired latch");
        in.eligible=true; grow(c,in,800,2000);
        check(c.state()==State::WarnLatched,"mode re-entry does not retrigger");
        c.disarm(); check(!c.latched() && c.state()==State::Disabled,"disarm resets latch");
        c.arm(cfg); grow(c,in,0,500); check(c.state()==State::Pending,"next arming takes new config");
    }
    {
        Core c; c.arm(config()); auto in=input();
        for (unsigned t=0;t<=2000;t+=10) {
            in.tilt_deg=t==150 ? 30 : 10;
            in.error_deg=t*0.01f;
            in.pwm[0]=t==250 ? 1949 : 1950;
            c.update(t,in);
            check(!c.latched(),"loss from interrupted saturation is forgotten");
        }
        in.tilt_deg=30; in.error_deg=21; c.update(2010,in);
        check(c.latched(),"late loss during long saturation triggers immediately");
    }
    {
        Core c; c.arm(config()); auto in=input();
        for (unsigned t=0;t<=500;t+=10) {
            in.tilt_deg=t==150 ? 30 : 10;
            in.error_deg=t*0.01f; c.update(t,in);
        }
        check(c.latched(),"loss need only occur during the same continuous interval");
    }
    {
        Core c; c.arm(config()); auto in=input(); in.enabled_motors=3;
        for (unsigned t=0;t<10000;t+=10) {
            const unsigned which=(t/400)%2;
            in.pwm[which]=2000; in.pwm[1-which]=1949;
            in.error_deg=t*0.001f; c.update(t,in);
            check(!c.latched(),"alternating motors do not combine their timers");
        }
    }
    {
        Core c; c.arm(config()); auto in=input();
        for (unsigned t=0;t<=600;t+=10) {
            in.tilt_deg=22; in.error_deg=t*0.01f; c.update(t,in);
        }
        check(!c.latched(),"tilt must strictly exceed 110 percent");
        in.tilt_deg=22.001f; in.error_deg=7; c.update(610,in);
        check(c.latched(),"just above tilt limit with growth triggers");
    }
    {
        Core c; c.arm(config()); auto in=input(); in.tilt_deg=0;
        uint32_t t=0;
        while (t<3000) {
            in.error_deg=float(t)*0.01f; c.update(t,in);
            if (t>=100) check(std::fabs(c.error_rate()-10)<0.001f,"100 ms interpolation across jitter");
            t+=1+(t%7);
        }
        in.error_deg=std::numeric_limits<float>::quiet_NaN(); c.update(t,in);
        check(c.error_rate()==0 && c.saturation_ms()==0,"invalid attitude resets detection");
    }
    {
        Core c; c.arm(config()); auto in=input(); grow(c,in,UINT32_MAX-200,500);
        check(c.latched(),"millisecond wrap retains interval arithmetic");
    }
    {
        Core c; c.arm(config()); auto in=input(); in.enabled_motors=uint32_t(1)<<31;
        in.pwm[31]=2000; grow(c,in,0,500);
        check(c.latched() && c.trigger_motor()==32,"highest supported motor bit is handled safely");
    }
    {
        Core c=pending(); auto in=input();
        check(c.wants_send(500),"first transmission due immediately");
        check(c.sent(500,true),"first send identifies mode-change boundary");
        check(!c.wants_send(749) && c.wants_send(750),"retry interval 250 ms");
        check(!c.sent(750,true),"retry cannot request another mode change");
        check(c.wants_send(1000),"third send due"); c.sent(1000,true);
        check(c.transmissions()==3 && !c.wants_send(1249),"three-send cap");
        c.update(1250,in);
        check(c.state()==State::TimedOut && c.latched(),"750 ms deadline preserves latch");
        c.ack(1260,1,25,6220,0,1,1,1,1);
        check(c.state()==State::TimedOut,"late ACK does not revive transaction");
    }
    {
        Core c=pending(); c.sent(500,false);
        check(c.transmissions()==0 && !c.wants_send(519) && c.wants_send(520),"no-route attempts paced, not counted as sent");
        auto in=input(); c.update(1250,in);
        check(c.state()==State::TimedOut && c.transmissions()==0,"no-route deadline");
    }
    {
        Core c=pending();
        c.ack(501,1,25,6220,0,1,1,1,1);
        check(c.state()==State::Pending,"unsolicited ACK before any transmission ignored");
        c.sent(501,true);
        c.ack(502,2,25,6220,0,1,1,1,1);
        c.ack(503,1,26,6220,0,1,1,1,1);
        c.ack(504,1,25,6210,0,1,1,1,1);
        c.ack(505,1,25,6220,0,255,190,1,1);
        check(c.state()==State::Pending,"wrong source, command and recipient ACKs ignored");
        c.ack(506,1,25,6220,0,1,1,1,1);
        check(c.state()==State::Accepted && !c.wants_send(1000),"matching ACK stops retries");
    }
    {
        Core c=pending(); c.sent(500,true); c.ack(510,1,25,6220,2,1,1,1,1);
        check(c.state()==State::Rejected && c.ack_result()==2,"denial terminates transaction");
        c=pending(); c.sent(500,true); c.ack(510,1,25,6220,5,1,1,1,1);
        check(!c.wants_send(750),"in-progress ACK suppresses retries");
        c.ack(900,1,25,6220,0,1,1,1,1); check(c.state()==State::Accepted,"in-progress followed by success");
        c=pending(); c.sent(500,true); c.ack(510,1,25,6220,0,0,0,1,1);
        check(c.state()==State::Accepted,"MAVLink 1 omitted target extensions supported");
    }
    {
        Core c=pending(); c.sent(500,true); c.manual_close(1,26);
        check(c.state()==State::Pending,"other hook command does not cancel");
        c.manual_close(1,25); check(c.state()==State::Cancelled && c.latched(),"manual close cancels retries, retains latch");
        c=pending(); c.sent(500,true); auto in=input(); in.flying=false; c.update(510,in);
        check(c.state()==State::Cancelled,"landing cancels pending transaction");
        c.disarm(); check(!c.latched() && !c.wants_send(1000),"disarm cancels and resets");
    }
    {
        std::mt19937 rng(167);
        // Negative controls with arbitrary motor outputs and high/oscillating error.
        for (unsigned run=0;run<200;run++) {
            Core c; auto cfg=config(run%2); c.arm(cfg); auto in=input();
            for (unsigned t=0;t<5000;t+=5) {
                in.enabled_motors=0xFFFFFFFFU;
                for (auto &pwm:in.pwm) pwm=uint16_t(1000+rng()%950);
                in.error_deg=float(rng()%18000)*0.01f;
                in.tilt_deg=float(rng()%9000)*0.01f;
                c.update(t,in);
                check(!c.latched() && !c.wants_send(t),"random outputs below threshold never release");
            }
        }
    }
    std::printf("PASS: %u production-core assertions\n",checks);
}
