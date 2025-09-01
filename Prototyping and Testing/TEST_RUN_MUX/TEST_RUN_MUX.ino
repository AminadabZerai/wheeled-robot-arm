/* ============================================================================
   Dual-Motor / Dual-Encoder DIAGNOSTIC (TCA9548A + AS5600 + TB6612FNG)
   Target   : Arduino Nano 33 IoT (3.3 V)
   Motor Pwr: 7.4 V
   Purpose  : Hardware & feedback validation without PID/FF. Safe, simple tests.
   ----------------------------------------------------------------------------
   Commands (type in Serial @115200):
     help
     scan
     al                // one LF angle sample
     ar                // one RF angle sample
     stream,al|ar|all  // stream angles @10 Hz
     rate,al|ar|all    // stream angle + velocity @10 Hz
     pwm,lf,<pwm>,<ms>     | pwm,rf,<pwm>,<ms>         (forward)
     pwmr,lf,<pwm>,<ms>    | pwmr,rf,<pwm>,<ms>        (reverse)
     sweep,lf|rf,<pwm_min>,<pwm_max>,<step>            (600ms per step)
     flip,lf|rf,<pwm>,<cycles>                         (500ms F/500ms R)
     muxstress,<cycles>   // rapid ch0/ch3 switching reads
     csv,rate,all         // CSV header + continuous rate stream (10 Hz)
     x                    // emergency stop/exit streaming
   ========================================================================= */

#include <Wire.h>
#include "AS5600.h"

// ----- Pins (TB6612FNG) -----
#define PWM2 9   // LF PWM  (B channel)
#define BIN1 7
#define BIN2 8

#define PWM1 5   // RF PWM  (A channel)
#define AIN1 3
#define AIN2 4

// ----- I2C -----
#define TCA_ADDR 0x70
#define AS_ADDR  0x36

// Encoders
AS5600 encLF;   // mux ch0
AS5600 encRF;   // mux ch3

// Helpers
inline void muxSelect(uint8_t ch){
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1u << (ch & 7));
  Wire.endTransmission();
}

uint16_t rawAngleOn(uint8_t ch, AS5600& enc){
  muxSelect(ch);
  delayMicroseconds(300);
  (void)enc.rawAngle();              // throw 1
  uint16_t a = enc.rawAngle();       // A
  uint16_t b = enc.rawAngle();       // B
  return (abs((int)a-(int)b) <= 2) ? a : b;
}

float rawToDeg(uint16_t raw){ return (raw * 360.0f / 4096.0f); }

// Motor drive
void motorCmdLF(int pwm, bool forward){
  analogWrite(PWM2, abs(pwm));
  if (pwm == 0){ digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW); return; }
  if (forward){  digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW); }
  else         { digitalWrite(BIN1,LOW);  digitalWrite(BIN2,HIGH); }
}
void motorCmdRF(int pwm, bool forward){
  analogWrite(PWM1, abs(pwm));
  if (pwm == 0){ digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW); return; }
  if (forward){  digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW); }
  else         { digitalWrite(AIN1,LOW);  digitalWrite(AIN2,HIGH); }
}
void allStop(){ motorCmdLF(0,true); motorCmdRF(0,true); }

// Angle unwrap state
float prevDegLF = 0.0f, prevDegRF = 0.0f;
bool primed = false;
float unwrap(float nowDeg, float &prevDeg){
  float d = nowDeg - prevDeg;
  if (d > 180.0f)  d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  prevDeg = nowDeg;
  return d;
}

// Streaming control
enum Mode {IDLE, STREAM_ANG_AL, STREAM_ANG_AR, STREAM_ANG_ALL,
           STREAM_RATE_AL, STREAM_RATE_AR, STREAM_RATE_ALL,
           CSV_RATE_ALL} mode = IDLE;
unsigned long lastTick = 0;
const unsigned long tickMs = 100; // 10 Hz

// ---- Utilities ----
void scanMux(){
  Serial.println(F("I2C scan per TCA9548A channel (expect AS5600 at 0x36):"));
  for (uint8_t ch=0; ch<8; ++ch){
    muxSelect(ch); delay(3);
    Wire.beginTransmission(AS_ADDR);
    uint8_t err = Wire.endTransmission();
    Serial.print(F("  CH")); Serial.print(ch); Serial.print(F(": "));
    Serial.println(err==0 ? F("FOUND 0x36") : F("-"));
  }
  muxSelect(0);
}

void printHelp(){
  Serial.println(F("Commands:"));
  Serial.println(F(" help | scan | al | ar"));
  Serial.println(F(" stream,al|ar|all"));
  Serial.println(F(" rate,al|ar|all"));
  Serial.println(F(" pwm,lf|rf,<pwm>,<ms>     (forward)"));
  Serial.println(F(" pwmr,lf|rf,<pwm>,<ms>    (reverse)"));
  Serial.println(F(" sweep,lf|rf,<min>,<max>,<step>  (600ms each)"));
  Serial.println(F(" flip,lf|rf,<pwm>,<cycles> (500ms F / 500ms R)"));
  Serial.println(F(" muxstress,<cycles>"));
  Serial.println(F(" csv,rate,all   (CSV header + stream)"));
  Serial.println(F(" x   (stop/exit)"));
}

// ---- Setup ----
void setup(){
  Serial.begin(115200);
  while(!Serial);

  Wire.begin();
  Wire.setClock(400000);

  pinMode(PWM1,OUTPUT); pinMode(PWM2,OUTPUT);
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
  allStop();

  // init encoders
  muxSelect(0); encLF.begin();
  muxSelect(3); encRF.begin();

  scanMux();

  // prime unwrap
  muxSelect(0); prevDegLF = rawToDeg(encLF.rawAngle());
  muxSelect(3); prevDegRF = rawToDeg(encRF.rawAngle());
  primed = true;

  Serial.println(F("DIAG ready. Type 'help'."));
}

// ---- Command parsing ----
void doPWM(const String& side, int pwm, int ms, bool forward){
  pwm = constrain(pwm, 0, 255);
  ms  = max(ms, 50);
  Serial.print(F("[PWM] ")); Serial.print(side);
  Serial.print(forward ? F(" FWD ") : F(" REV "));
  Serial.print(F("pwm=")); Serial.print(pwm);
  Serial.print(F(" ms=")); Serial.println(ms);

  if (side=="lf") motorCmdLF(pwm, forward);
  else            motorCmdRF(pwm, forward);
  delay(ms);
  allStop();
}

void doSweep(const String& side, int pmin, int pmax, int step){
  pmin = constrain(pmin, 0, 255);
  pmax = constrain(pmax, 0, 255);
  step = max(step, 1);
  if (pmin > pmax) { int t=pmin; pmin=pmax; pmax=t; }

  Serial.print(F("[SWEEP] ")); Serial.print(side);
  Serial.print(F(" ")); Serial.print(pmin);
  Serial.print(F("..")); Serial.print(pmax);
  Serial.print(F(" step ")); Serial.println(step);

  for (int p=pmin; p<=pmax; p+=step){
    Serial.print(F("  pwm=")); Serial.println(p);
    if (side=="lf") motorCmdLF(p, true);
    else            motorCmdRF(p, true);
    delay(600);
  }
  allStop();
}

void doFlip(const String& side, int pwm, int cycles){
  pwm = constrain(pwm, 0, 255);
  cycles = max(cycles, 1);
  Serial.print(F("[FLIP] ")); Serial.print(side);
  Serial.print(F(" pwm=")); Serial.print(pwm);
  Serial.print(F(" cycles=")); Serial.println(cycles);
  for(int i=0;i<cycles;i++){
    if (side=="lf"){ motorCmdLF(pwm, true);  delay(500); motorCmdLF(pwm, false); delay(500); }
    else           { motorCmdRF(pwm, true);  delay(500); motorCmdRF(pwm, false); delay(500); }
  }
  allStop();
}

void doMuxStress(int cycles){
  cycles = max(cycles, 1);
  Serial.print(F("[MUXSTRESS] cycles=")); Serial.println(cycles);
  for (int i=0;i<cycles;i++){
    uint16_t r0 = rawAngleOn(0, encLF);
    uint16_t r3 = rawAngleOn(3, encRF);
    if ((i%25)==0){
      Serial.print(F("  i=")); Serial.print(i);
      Serial.print(F(" raw0=")); Serial.print(r0);
      Serial.print(F(" raw3=")); Serial.println(r3);
    }
  }
}

void startCSVRateAll(){
  mode = CSV_RATE_ALL;
  Serial.println(F("t_s,deg_LF,vel_LF,deg_RF,vel_RF"));
}

void handleCommand(String cmd){
  cmd.trim(); cmd.toLowerCase();
  if (cmd=="help"){ printHelp(); return; }
  if (cmd=="x"){ allStop(); mode=IDLE; Serial.println(F("[STOP]")); return; }
  if (cmd=="scan"){ scanMux(); return; }
  if (cmd=="al"){ float d=rawToDeg(rawAngleOn(0,encLF)); Serial.print(F("[AL] ")); Serial.println(d,2); return; }
  if (cmd=="ar"){ float d=rawToDeg(rawAngleOn(3,encRF)); Serial.print(F("[AR] ")); Serial.println(d,2); return; }

  if (cmd=="stream,al"){ mode=STREAM_ANG_AL; return; }
  if (cmd=="stream,ar"){ mode=STREAM_ANG_AR; return; }
  if (cmd=="stream,all"){ mode=STREAM_ANG_ALL; return; }

  if (cmd=="rate,al"){ mode=STREAM_RATE_AL; return; }
  if (cmd=="rate,ar"){ mode=STREAM_RATE_AR; return; }
  if (cmd=="rate,all"){ mode=STREAM_RATE_ALL; return; }

  if (cmd=="csv,rate,all"){ startCSVRateAll(); return; }

  // pwm,lf,120,1500  /  pwmr,rf,120,1500
  if (cmd.startsWith("pwm,")){
    int c1 = cmd.indexOf(',',4);
    int c2 = cmd.indexOf(',',c1+1);
    if (c1<0 || c2<0) { Serial.println(F("Usage: pwm,lf|rf,<pwm>,<ms>")); return; }
    String side = cmd.substring(4,c1);
    int pwm = cmd.substring(c1+1,c2).toInt();
    int ms  = cmd.substring(c2+1).toInt();
    doPWM(side,pwm,ms,true);
    return;
  }
  if (cmd.startsWith("pwmr,")){
    int c1 = cmd.indexOf(',',5);
    int c2 = cmd.indexOf(',',c1+1);
    if (c1<0 || c2<0) { Serial.println(F("Usage: pwmr,lf|rf,<pwm>,<ms>")); return; }
    String side = cmd.substring(5,c1);
    int pwm = cmd.substring(c1+1,c2).toInt();
    int ms  = cmd.substring(c2+1).toInt();
    doPWM(side,pwm,ms,false);
    return;
  }

  // sweep,lf,40,200,30
  if (cmd.startsWith("sweep,")){
    int c1 = cmd.indexOf(',',6);
    int c2 = cmd.indexOf(',',c1+1);
    int c3 = cmd.indexOf(',',c2+1);
    if (c1<0 || c2<0 || c3<0){ Serial.println(F("Usage: sweep,lf|rf,<min>,<max>,<step>")); return; }
    String side = cmd.substring(6,c1);
    int pmin = cmd.substring(c1+1,c2).toInt();
    int pmax = cmd.substring(c2+1,c3).toInt();
    int step = cmd.substring(c3+1).toInt();
    doSweep(side,pmin,pmax,step);
    return;
  }

  // flip,rf,120,6
  if (cmd.startsWith("flip,")){
    int c1 = cmd.indexOf(',',5);
    int c2 = cmd.indexOf(',',c1+1);
    if (c1<0 || c2<0){ Serial.println(F("Usage: flip,lf|rf,<pwm>,<cycles>")); return; }
    String side = cmd.substring(5,c1);
    int pwm = cmd.substring(c1+1,c2).toInt();
    int cyc = cmd.substring(c2+1).toInt();
    doFlip(side,pwm,cyc);
    return;
  }

  // muxstress,200
  if (cmd.startsWith("muxstress,")){
    int c1 = cmd.indexOf(',',10);
    if (c1<0){ Serial.println(F("Usage: muxstress,<cycles>")); return; }
    int cyc = cmd.substring(c1+1).toInt();
    doMuxStress(cyc);
    return;
  }

  Serial.println(F("Unknown command. Type 'help'."));
}

// ---- Loop ----
void loop(){
  // read commands
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // streaming every tick
  unsigned long now = millis();
  if (mode!=IDLE && now - lastTick >= tickMs){
    lastTick = now;

    uint16_t r0 = rawAngleOn(0, encLF);
    uint16_t r3 = rawAngleOn(3, encRF);
    float d0 = rawToDeg(r0);
    float d3 = rawToDeg(r3);

    if (!primed){ prevDegLF=d0; prevDegRF=d3; primed=true; }

    float dt = tickMs / 1000.0f;
    float dd0 = unwrap(d0, prevDegLF);
    float dd3 = unwrap(d3, prevDegRF);
    float v0 = -dd0 / dt;   // sign convention: match your controller (−Δθ/Δt)
    float v3 = -dd3 / dt;

    switch(mode){
      case STREAM_ANG_AL:  Serial.print(F("[AL] ")); Serial.println(d0,2); break;
      case STREAM_ANG_AR:  Serial.print(F("[AR] ")); Serial.println(d3,2); break;
      case STREAM_ANG_ALL: Serial.print(F("[ALL] LF=")); Serial.print(d0,2);
                           Serial.print(F(" RF=")); Serial.println(d3,2); break;
      case STREAM_RATE_AL: Serial.print(F("[AL] deg=")); Serial.print(d0,2);
                           Serial.print(F(" vel=")); Serial.println(v0,2); break;
      case STREAM_RATE_AR: Serial.print(F("[AR] deg=")); Serial.print(d3,2);
                           Serial.print(F(" vel=")); Serial.println(v3,2); break;
      case STREAM_RATE_ALL:Serial.print(F("[ALL] LF(deg,vel)=")); Serial.print(d0,2);
                           Serial.print(F(",")); Serial.print(v0,2);
                           Serial.print(F("  RF(deg,vel)=")); Serial.print(d3,2);
                           Serial.print(F(",")); Serial.println(v3,2); break;
      case CSV_RATE_ALL:   Serial.print(now/1000.0f,3); Serial.print(',');
                           Serial.print(d0,3); Serial.print(',');
                           Serial.print(v0,3); Serial.print(',');
                           Serial.print(d3,3); Serial.print(',');
                           Serial.println(v3,3); break;
      default: break;
    }
  }
}
