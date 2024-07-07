/*
 Title       NS302BT
 by          Nishioka Sadahiko
 *

 Copyright (c) 2024 Nishioka Sadahiko

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

 *Description:
 *Encoder-based telescope pointing microcontroller with Bluetooth,serial（RS232C) interfaces, 
 *for equatorial and other mounts, and LX200 compatible command set.

 * Author: Nishioka Sadahiko
 *   https://nskikaku.sakura.ne.jp/index.html
 *   https://www.facebook.com/nishioka.sst
 *   nishioka.sst@gmail.com
*/

/****************************************************************
    Digital Observer Guide
    NS302BT Digital Setting Circles Ver1.1 2019,2/2 2019,3/6

//Ver1.01
   void _bluetooth_muart()　の timeoutz -->timeoutbz へ訂正　2019.1/27

//Ver1.00
    void  calc_RA_low() 低精度を作る　2019,1/17　
    void  calc_DEC_low() 低精度　2019,1/17
    //_serialMode1 = NS5000; // NS5000モードを禁止する　2019,1/13
    //_serialMode2 = NS5000; // NS5000モードを禁止する　2019,1/13
    //アライメント要求入力  割り込み 禁止する　2019,1/14

    NS-302 USB bluetooth Navigator Ver1.0 を
    NS302_ESP32_Navigator
    NS5000BT_unit
    NS5000WiFi_unit   にコピーした。2018,12/17
    
   (C)2018 NS_KIKAKU
   
****************
  基板:　   ESP32S Dev Module
  arduino:  ESP32 Dev Module
****************

  Ver 1.01   NS-302 USB & bluetooth ESP32 Navigatorをベースにする。2018,11/27

            NS-302 USB & bluetooth ESP32 Navigator  2018,11/12 -- 2018,11/15 -- 2018,11/25 2018,11/26
            NS-302 bluetooth ESP32 Navigator  2018,11/12
            hSerial1が受信異常、入力端子をINPUT_PULLUPしたら正常に動いた。2018,11/14
            アライメント要求入力実現した。 2018,11/15
            BTssid bluetoothデバイス名を登録した。2018,11/16
            Timer,外部割り込みの後ではEEPROM命令（RESETがかかる）が使えないため、
            SPIFFS filesystem関連コマンドを作った。2018,11/22 11/23
             :@**format#  :@**newfile#を追加した2018,11/24
             電源投入時、初期化ファイル読み込み機能を追加した2018,11/24
             spiffs filename文字数を修正した、ファイル名は１２文字まで。例12345678.txt 2018,11/26
             

 
****************
  基板:　   NodeMCU 1.0 (ESP-12E Module)
  arduino:  NodeMCU 1.0 (ESP-12E Module)
  NS-302 USB nano Navigator  2018,6/15 -- 8/9、をベースにして移植した。2018,8/9
*****************
  基板:　  arduino UNO ,arduino NANO
　         注意、encoder 分解能　1:x1　のみ有効である　2018,8/9

  Vwe 1.0   2018,7/14 1チャンネルだけとしSoftSerialを削除する。
            タイマー割り込み_msidereal_timer() は１秒に変更、恒星時を標準時で代用する。
            MsTimer2,EEPROM追加、修正した。
            以上により、arduino UNO　,NANOにも対応できる。
            UNO，NANOは割り込み端子は２しかないため、encoder 分解能　1:x1　のみ有効である。
            パラメータ保存に関係するコマンドは直ちにROM書き込みをするようにした。NS-302プロトコル一覧表を参照せよ。
 
　Ver 1.0   CPUはESP8266MODである。基板 WeMos D1 mini
　　　　　　2018,7/6　WiFi関係の変数、定義、プログラムを削除した。
            2018,7/7  :@*ModeX# にアンサOK#を入れ、直ちにROMに書き込むことにした。
            sysMode=2 でも赤道儀、経緯台コマンドを受け入れるようにした。
            赤経、赤緯の計算不具合を修正した。（整数、実数　混合計算）
　　　　　　　exe_inputDECxxxx_command()
　　　　　　　exe_inputRAxxxx_command()
　　　　　　　calc_RA()
　　　　　　　calc_DEC()
            2018,7/12 NS5000コマンド :=GVP# :=GVN# :*V# :=SD#(LED点滅)  :/NEW# を追加した。
            
**
  NS-301_Basic_Encoder_digital_display  2016,8/26, 9/27 -- 10/16 をベースにして開発する。2018,6/15
  Ver1.0  2016,10/16  SkySafari plusのBasic Encoder Systemプロトコルを実装した。10/13
                      _Telnet2_muart()のtimeoutzをtimeout2zに修正した。10/22
  
                    
**
  NS-401 WiFi-Serial Ver1.0 2016,9/17 -- 9/27 これをベースにして開発する。
    
****************************************************************/

         
#include <Arduino.h>
#include <Ticker.h>
//#include <SoftwareSerial.h>
//#include <EEPROM.h>
#include <BluetoothSerial.h>
#include "FS.h"
#include "SPIFFS.h"


//***********************************************************
//  NodeMCU 1.0 (ESP-12E Module) pin番号定義  CPUはESP8266MODである。

// NS302 ENCODER入力
#define DEC_A              33
#define DEC_B              32
#define RA_A               35
#define RA_B               34


// NS302  ランプ
#define TLED               25    // 赤
#define LED1               26    // 白 serial1
#define LED2               27    // 緑 serial2


#define sU1RX              18
#define sU1TX              19    //soft Serial 


//　アライメント要求入力 2018,11/15
#define  _input_Alignment  12


//***********************************************************
// Serial 0                      // UART0 USB (RX=3, TX=1)

HardwareSerial  hSerial1(1);     // UART1    ピンを変更しなければならない
//ESP-WROOM32 には IO6～IO11, IO34, IO35は使用不可

HardwareSerial hSerial2(2);      // UART2 (RX=16, TX=17)

BluetoothSerial bSerial;       //  bluetooth Serial



//***********************************************************
//   定義
//***********************************************************
#define  VERSION            "NS302BT Digital Setting Circles Ver1.1#"
#define  GVP                "NS302BT#"
#define  GVN                "1.11#"
#define  COPYRIGHT          "(C)2019 NS_KIKAKU#"
#define  OK                 1
#define  NG                 0



//***********************************************************
//   マクロ
//***********************************************************
// TLED
#define  bsetTLED            digitalWrite(TLED, HIGH);
#define  bclrTLED            digitalWrite(TLED, LOW)

#define  okLED               tledtimerz =60; 
#define  startLED            tledtimerz =200; 
#define  guideLED            tledtimerz =5; 
#define  startAP_MODE_LED    tledtimerz =1000;

// LEDmode1/2         2018,11/14追加
#define  bsetLED1     digitalWrite(TLED, HIGH);   //NS5000 モード 点灯
#define  bclrLED1     digitalWrite(TLED, LOW);    //S302 モード

#define  bsetLED2     digitalWrite(TLED, HIGH);   //NS5000 モード 点灯
#define  bclrLED2     digitalWrite(TLED, LOW);    //S302 モード



//***********************************************************
//  EEPROMメモリアドレス定義 
//***********************************************************
#define  EEPROM_SIZE        256
#define  EEPROM_OK          195               //登録済・有効

#define  ROM_romsw                    0x0000  // 1byte  EEPROM_OK:登録済・有効　    それ以外:未使用・未登録である
#define  ROM_sysMode                  0x0003  // 1byte  *** モード　０：赤道儀　１：経緯台　２：NEXUS互換機

#define  ROM_encoder_maxRA            0x0010  // 4byte ***encoder 基本定数  １回転分のencoder周回パルス
#define  ROM_encoder_resolutionRA     0x0014  // 1byte ***encoder 分解能　1:x1   2:x2　  4:x4 逓倍
#define  ROM_encoder_invertRA         0x0015  // 1byte ***encoder CW-CCW反転　0:通常　1:CW-CCWを反転する

#define  ROM_encoder_maxDEC           0x0020  // 4byte ***encoder 基本定数  １回転分のencoder周回パルス
#define  ROM_encoder_resolutionDEC    0x0024  // 1byte ***encoder 分解能　1:x1   2:x2　  4:x4 逓倍
#define  ROM_encoder_invertDEC        0x0025  // 1byte ***encoder CW-CCW反転　0:通常　1:CW-CCWを反転する

#define  ROM_serialMode1              0x0030  // 1byte serial1 モード　NS302 NS5000 2018,11/15 追加
#define  ROM_serialMode2              0x0031  // 1byte serial2 モード　NS302 NS5000

#define  ROM_BTssid                   0x0040  //*** SSID_Bluetooth 2018,11/16追加


//***********************************************************
//   グローバル変数
//***********************************************************
int _pcsw;

char* p;
int result = NG;                          //コマンドの実行結果を入れる　OK or NG

char CR[]="\r";

#define  inSERIAL           0             // UB (USB)
#define  inBLUETOOTH        1             // BT (BlueTooth)
#define  inWIFI             2             //
#define  inWEBSERVER        3             //
#define  outSERIAL1         4             // S1 (Serial1)
#define  outSERIAL2         5             // S2 (Serial2)
int _insel2_sw = inBLUETOOTH;

#define  NS5000             0             //シリアル１，２のモード
#define  NS302              1
byte  _serialMode1     = NS302;           //デフォルト
byte  _serialMode2     = NS302;           //

boolean  _ns302Aligned = false;          // true アライメント完了した





//***********************************************************  NS302
//   system モード
byte  romsw   = 0;                        //未登録　　　EEPROM_OK 以外は未登録

byte  sysMode = 0;                        //*** ０：赤道儀　１：経緯台　２：NEXUS互換機
                                          //NEXUS(Digital Setting Circles)

//***********************************************************
//  _muartx()  hard serial 0
//
byte muartport =0;                //working

#define CMD_BUF_SIZE 1000         //受信したコマンドをcopyしておく。@___#＋NULL  終端に０を付ける。
#define TIMEOUT    500            //コマンド Timeout * 10ms

int timeoutz   =0;                //  Serial (USB)
int timeout1z  =0;                //  hSerial1
int timeout2z  =0;                //  hSerial2
int timeoutbz  =0;                //  bSerial (bluetooth)

//******  exe_command()
char  *buf;                        // getdt() bufから、文字を取り出す。
int    buf_pnt;                    // getdt()
int    buf_cnt;

//***********************************************************
// TLED
int tledtimerz;                    //点滅時間　ledtimer *10ms　ダウンカウンタ ->0

int tledstate =LOW;                //0:消灯  1:点灯
#define TLEDBLINK_TIME  7          //点滅周期を決める
int blinkz =TLEDBLINK_TIME;        //点滅周期　10ms
int tledstate_off_time =LOW;       //点滅以外の状態を指定する


//***********************************************************
// Timer割り込み
Ticker Ticker_timer;
int timer10ms =0;                  // 10ms upタイマ
int timer1s   =0;                  // 1s upタイマ
int t10       =0;                  // working
boolean bf_10ms =false;            // 10ms 処理 頃にcalc_Basic_Encoder_System() を実行する。


//******************************************************************************
//   恒星時　sidereal time　割り込み  997ms
Ticker Ticker_sidereal_timer;
unsigned long sidereal_timer =0;     //恒星時997ms upタイマー


//***********************************************************
// azimuth方位　RA ENCODER
byte encoder_resolutionRA    =2;                // ***encoder 分解能　1:x1   2:x2　  4:x4 逓倍
byte encoder_invertRA        =0;                // ***encoder CW-CCW反転　0:通常　1:CW-CCWを反転する
unsigned long  encoder_maxRA =10000;            // ***encoder 基本定数  １回転分のencoder周回パルス

unsigned long  encoder_cntRA =0;                // ***　角度を表す　encoderカウンタ 　0 ～ encoder基本定数－１　までの値をとる
byte encoder_stateRA         =0;                // A,B相の状態　　bit1:A相　　bit0:B相


//***********************************************************
// altitude 高度　DEC ENCODER
byte encoder_resolutionDEC    =2;               // ***encoder 分解能　1:x1   2:x2　  4:x4 逓倍
byte encoder_invertDEC        =0;               // ***encoder CW-CCW反転　0:通常　1:CW-CCWを反転する
unsigned long  encoder_maxDEC =10000;           // ***encoder 基本定数  １回転分のencoder周回パルス

unsigned long  encoder_cntDEC =0;               // ***　角度を表す　encoderカウンタ 　0 ～ encoder基本定数－１　までの値をとる
byte encoder_stateDEC         =0;               // A,B相の状態　　bit1:A相　　bit0:B相
byte f_cross_center           =LOW;             // 赤緯、高度が、90°,180°,12時を超えた場合を判定する　それにより赤経、方位を更新する


//***********************************************************
// 方位、高度、赤経、赤緯
char  sAZangle[10];                             // 方位　XXX.xxx
float fAZ;
char  sALTangle[10];                            // 高度　±XX.xxx
float fALT;

char  sRAhour[12];                              // 赤経  hh:mm:ss#
char  sDECdhour[12];                            // 赤緯  dd:mm:ss#

char  sSign;                                    // 赤緯　符号   


//***********************************************************
// Basic Encoder System

#define  encoder_Basic 100000             // basic encoder system のencoder_cntの基準
char basic_encoder_system_Response[16];   // Basic Encoder SystemのResponseを格納する 例　+00020\t-00300\r 


//*********************************************************** 2018,11/15追加
//   Alignment要求
boolean  f_alignment  = false;      // アライメント有効
int alignment_timerz  = 100;        // アライメントtimer １秒後に実行する


// bluetooth SSID
char  BTssidname[25]  = "NS302";   //NS302-Helloと表示される
char* BTssid          = BTssidname + 5;


// SPIFFS
File file;
#define FILE_NAME_MAX   13               //ファイル名（拡張子含む）は13文字以下
                                         //例　filename . txt　8+1+3　14文字以上ではresetした　2018,11/23
#define FORMAT_SPIFFS_IF_FAILED true

char ns302renew_file_name[16] =  "/NS302NEW.txt";  //NS302 初期化ファイル名
char* ns302rfn = ns302renew_file_name;


// NS302,シリアル1,2 赤経、赤緯
// RA1,DEC1/Serial1  RA2,DEC2/Serial2  RA302,DEC302/NS302  2018,12/8






//***********************************************************  /NS302






//*********************************************************** setup
//  初期設定
//*********************************************************** 
void setup() {

  
//***************************** inz output Port
  pinMode(TLED,  OUTPUT);
  bclrTLED;
  pinMode(LED1,  OUTPUT);     //serial1 点灯 NS5000モード
  bclrLED1;
  pinMode(LED2,  OUTPUT);     //serial2 点灯 NS5000モード
  bclrLED2;


//***************************** inz input Port
  pinMode( _input_Alignment,INPUT);         // アライメント要求入力
  pinMode( _input_Alignment, INPUT_PULLUP);

  
//***************************** inz ENCODER
  pinMode(DEC_A,  INPUT);     // A相 
  pinMode(DEC_B,  INPUT);     // B相 
  pinMode(RA_A,   INPUT);
  pinMode(RA_B,   INPUT);
  
  pinMode(DEC_A, INPUT_PULLUP);
  pinMode(DEC_B, INPUT_PULLUP);
  pinMode(RA_A,  INPUT_PULLUP);
  pinMode(RA_B,  INPUT_PULLUP);

 
//***************************** inz EEPROM
// パラメータを初期化する。
// デフォルトパラメータは設定済みの状態である。
// これにEEPROMのパラメータを上書きする。
  //EEPROM.begin(EEPROM_SIZE);

 //if (EEPROM.read(ROM_romsw) == EEPROM_OK) {
 //    read_EEPROM();
 //}



  
//***************************** inz Serial port
 // inz hard Serial 0
  Serial.begin(9600, SERIAL_8N1);

  // inz hard Serial 1
  hSerial1.begin(9600, SERIAL_8N1, 18, 19);  // ピンを変更 (RX=18, TX=19)
  //hSerial1.begin(19200, SERIAL_8E1, 18, 19);
  pinMode(18, INPUT_PULLUP);
  //参考　IO34～IO39はプルアップなし
  //https://qiita.com/no_clock/items/a3bc8a9816534cf8c930
  
  // inz hard Serial 2
  hSerial2.begin(9600, SERIAL_8N1);
  //hSerial2.begin(19200, SERIAL_8E1);
  
  // bluetooth Serial
  //bSerial.begin(BTssidname);


//***********************************************************
//  SPIFFS file system
/*
    if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
            Serial.println("SPIFFS Mount Failed");
        return;
    }
*/

    
//***************************** setup Basic Encoder System
// Basic Encoder System encoder_cntを準備する
  if ( sysMode == 2) setup_Basic_Encoder_System();  // softSerial  0:赤道儀 1:経緯台 2:NEXUS互換機


//***************************** inz Encoder
  encoder_stateRA = readEncoderRA();            //encoderの状態を読む
  if (encoder_resolutionRA == 1){
     attachInterrupt(digitalPinToInterrupt(RA_A), _mencoderRA, RISING);      // x1逓倍
  }else{
     attachInterrupt(digitalPinToInterrupt(RA_A), _mencoderRA, CHANGE);      // x2,x4逓倍
     attachInterrupt(digitalPinToInterrupt(RA_B), _mencoderRA, CHANGE);
  }

  encoder_stateDEC = readEncoderDEC();            //encoderの状態を読む
  if (encoder_resolutionDEC == 1){
     attachInterrupt(digitalPinToInterrupt(DEC_A), _mencoderDEC, RISING);    // x1逓倍
  }else{
     attachInterrupt(digitalPinToInterrupt(DEC_A), _mencoderDEC, CHANGE);    // x2,x4逓倍
     attachInterrupt(digitalPinToInterrupt(DEC_B), _mencoderDEC, CHANGE);   
  }


//***********************************************************
//   Timer割り込み
  Ticker_timer.attach_ms(10, _mtimer);


//***********************************************************
//   恒星時　sidereal time　割り込み  997ms
  Ticker_sidereal_timer.attach_ms(997, _msidereal_timer);


//***********************************************************
//   アライメント要求入力  割り込み
  //attachInterrupt(digitalPinToInterrupt(_input_Alignment), _AlignmentNS5000, FALLING);

  Serial.print("\n");
  Serial.print(VERSION);
  
  tledtimerz = 300;
  _pcsw =0;  

} // setup()
//*********************************************************** /setup






//*********************************************************** main loop
//  main loop
//*********************************************************** 
void loop() {
  static int pcflag =0;
  char  rxdt;
   
  switch(pcflag){
   
   case 0:            //SPIFFS有効　2018,11/24
     //  SPIFFS file system
     if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
         Serial.println("SPIFFS Mount Failed");
         pcflag = 3;
         break;
      }
      pcflag = 1;
      break;
     
   case 1:            //NS302初期化データファイルをOpenする
      file = SPIFFS.open(ns302rfn);
      if(!file || file.isDirectory()){
         Serial.println("- failed to open file for reading");
         pcflag = 3;
      }
      pcflag = 2;
      break;
     
   case 2:           //NS302を初期設定する、NS302NEW.txtから初期化データを読み込む
       if (!_renew_muart()) {
         //Serial.println("- failed to read from NEWfile");
         //pcflag = 3;
         break;
       }
       Serial.println("- read from NEWfile");
       
       encoder_cntDEC =encoder_maxDEC / 2; //デフォルトで天の北極にする　2019,3/19
       pcflag = 3;
       break;
     
   case 3:           //初期設定に基づいて、setupを実行し、所定のmain loopへ分岐する
       bSerial.begin(BTssidname);          // bluetooth Serialスタート

       pcflag = 4;
       break;
        
   case 4:           // bluetooth main loop
       _muart();                           // Serial(USB) コマンドを受信してから実行する。
       _bluetooth_muart();                 // bluetoothSerial  コマンドを受信してから実行する。

       if ((hSerial1.available() != 0) && ( _serialMode1 == NS5000)) {
          rxdt = hSerial1.read();           // Serial1 >> Serial
          Serial.print(rxdt);
       }

       if ((hSerial2.available() != 0) && ( _serialMode2 == NS5000)) {
          rxdt = hSerial2.read();           // hSerial2 >> bSerial
          bSerial.print(rxdt);
       }
       break;
       
   case 5:           // wifiserial main loop
       break;

   case 6:           // server main loop
       break;     
  }

 
  if (bf_10ms == true) {
    bf_10ms = false;
    
    // Basic Encoder System  座標を計算し、Responseを準備する
    calc_Basic_Encoder_System();      

    // serial1/2 モードによりLED1,LED2　点灯させる　2018,11/15 追加
    if ( _serialMode1 == NS5000 ) { bsetLED1; }else{ bclrLED1; }
    if ( _serialMode2 == NS5000 ) { bsetLED2; }else{ bclrLED2; }
    
    // serial 1,2   Alignment要求を実現する
    exe_AlignmentNS5000();            
  }
}
//*********************************************************** /main loop





//*********************************************************** / _Alignment　  2018,11/16
//   Alignmentを実行する 
//***********************************************************
void exe_AlignmentNS5000() {

    //   serial1,seria2　赤道儀を停止させる。 
    if (( f_alignment  == true ) && (alignment_timerz  == 450)) {
      hSerial1.print("#:Q#");
      hSerial2.print("#:Q#");
    }
      
    //   serial 1 Alignmentを実現する 
    if (( f_alignment  == true ) && (alignment_timerz  == 100)) {
      
      //  :SrHH:MM:SS# 送信
      Serial.print("#:Sr"); //************
      hSerial1.print("#:Sr");
      calc_RA();
      strcat(sRAhour,"#");
      Serial.print(sRAhour); //************
      hSerial1.print(sRAhour);
      
      //  :SdsDD*MM:SS# 送信
      Serial.print("#:Sd"); //************
      hSerial1.print("#:Sd");
      calc_DEC();
      strcat(sDECdhour,"#");
      Serial.print(sDECdhour); //************
      hSerial1.print(sDECdhour);
      
      Serial.print(":CM#");    //************
      hSerial1.print(":CM#");
    }

    //   serial 2 Alignment要求を実現する 
    if (( f_alignment  == true ) && (alignment_timerz  == 0)) {
      f_alignment  = false;
      
      //  :SrHH:MM:SS# 送信
      Serial.print("#:Sr"); //************
      hSerial2.print("#:Sr");
      calc_RA();
      strcat(sRAhour,"#");
      Serial.print(sRAhour); //************
      hSerial2.print(sRAhour);
      
      //  :SdsDD*MM:SS# 送信
      Serial.print("#:Sd"); //************
      hSerial2.print("#:Sd");
      calc_DEC();
      strcat(sDECdhour,"#");
      Serial.print(sDECdhour); //************
      hSerial2.print(sDECdhour);

      Serial.print(":CM#");    //************
      hSerial2.print(":CM#");
    }
}



//*********************************************************** / _Alignment　  2018,11/15
//   Alignment要求を指示する
//***********************************************************
void _AlignmentNS5000(){
   
   if ( _ns302Aligned != true ) return;   // true アライメント完了していなければ、無視する
   f_alignment       = true;              // アライメント実行せよ
   alignment_timerz  = 500;               // １，２秒後にシリアル１，２をアライメントする
   tledtimerz        = 500;
}



//*********************************************************** / Timer
//   Timer割り込み  10ms
//***********************************************************
//   timer10ms :　　10ms upタイマー
//   timer1s   :　　1s  upタイマー

void _mtimer(){
  
    //*********** 10ms upタイマ
    timer10ms++;



    bf_10ms = true;                    // 10ms 毎にcalc_Basic_Encoder_System() を実行する。

    //**** タイムアウト->0
    if (timeoutz  !=0) timeoutz--;
    if (timeoutbz !=0) timeoutbz--;
    if (timeout1z !=0) timeout1z--;
    if (timeout2z !=0) timeout2z--;

    // アライメントtimer
    if (alignment_timerz != 0)  alignment_timerz--;  
    
    //****  blink時間
    if (tledtimerz !=0) {
       tledtimerz--;
           
       blinkz--;
       if (blinkz == 0){
          blinkz =TLEDBLINK_TIME;      //点滅周期
          tledstate = !tledstate;      //**** TLED　点滅コントロール
          //dspTLED(tledstate);
       }
    }else{
       tledstate = tledstate_off_time;
    }

    t10++;
    // 1秒タイマ upタイマ
    if (t10 == 100)  {
       timer1s++;
       t10 =0;
    }
    
    //  TLEDを表示する
    if (tledstate == 1) {
       bsetTLED;
    }else{
       bclrTLED;
    }
}



//*********************************************************** /sidereal time
//   恒星時　sidereal time　割り込み  997ms
//***********************************************************
//   sidereal_timer :　　　恒星時 upタイマー
void _msidereal_timer(){
   
    sidereal_timer++;
    if (sidereal_timer == 86400) sidereal_timer =0;
    
  // Serial.print("\n");
  //  Serial.print(sidereal_timer);
    
}
