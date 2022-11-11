/*
 Kiln Temperature controller WiFi Web Server.
 
 Modbus ASCII examples for Solo SL4848.
 Read Holding Register(s) example  <: start char><01 id><03 code><1000 reg><0008 registers><LRC>
 Write Coil example :010508140000DE <: start char><01 id><05 code><0814 reg><0000 clear, FF00 set data><LRC>
 Write single Holding Register example :010610300001B8  <: start char><01 id><06 code><1030 reg><0001 data><LRC>
 Write multiple Holding Register example :01102000000810271007D00BB80FA0138817701770177007
                                            <: start char><01 id><10 code><2000 reg><0008 registers><10 bytes><2710 data1><07D0 data2><etc><LRC>

TODO:
-Save runs  (test new point n, if n-1 is on line n to n-2, discard n-1)
-Have tabs, one is Saved runs - left tab list runs, right tab brings up a run.
-Have buttons for delete run, re-run.
-Add run name to main tab
-Settings tab -- kiln name.
-Add pause button
-Add 'advance' or 'retard' to next/previous step
-fixed  --if refresh webpage, graph starts at 0, not at actual time.
-Save state--so if repower in middle of a run, pick up at last point.
*/

#include <string>
#include <WiFi.h>
#include "time.h"
#include <sys/time.h>
#include "apps/sntp/sntp.h"

#include <HardwareSerial.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define SLAVE_ID 1
#define REGISTERS_MAX_LIMIT 8
#define READ_COIL 0x01
#define READ_HOLDING_REGISTER 0x03
#define WRITE_SINGLE_REGISTER 0x06
#define WRITE_MULTIPLE_REGISTERS 0x10
#define WRITE_SINGLE_COIL 0x05
#define REGISTER_PROBE_TYPE 0x1004
#define REGISTER_CONTROL_MODE 0x1005
#define REGISTER_RUN_STOP 0x0814
#define REGISTER_HOLD 0x0816
#define RESPONSE_BYTES_REGISTER 2
#define RESPONSE_BYTES_COIL 1

#define CHECK_TEMP_EVERY_SECS 6
/*




#define RXD2 45
#define TXD2 46
*/

#define RXD2 16
#define TXD2 17




const char* ssid     = "Coldpops";
const char* password = "reneerenee";

const char* ntpServer = "pool.ntp.org";
//const long  gmtOffset_sec = 0;
//const int   daylightOffset_sec = 3600;

HardwareSerial HardS(1);
AsyncWebServer server(80);

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

char line[100];
uint16_t html_next = 0;

typedef struct {
  time_t timestamp;
  uint16_t pv; // PV       1000
  uint16_t sv; //Target SV 101D
  uint8_t leds; //LED status
} state_t;
//state_t state[6500];
state_t *state; //Moved to PSRAM.

int state_n_elements;
int state_n = 0;
int state_sent_n = 0;

struct {
  time_t run_start_time = 0;
  uint8_t control_mode = 3; // 0 = PID control, 1 = On / Off control, 2 = Manual control, 3 = Ramp / Soak
  uint8_t probe_type = 20; // 0 = K type. 0 - 17 valid.
  uint8_t run_stop = 0; //0 stop, 1 run.
  uint8_t hold = 0; // 1 hold, 0 run.
} state_current;

time_t now;
int tz_offset_secs;
time_t prev_time;
time_t start_time;
struct tm timeinfo;
char strftime_buf[64];

int bytes;
char req_str[60];

typedef struct {
  uint8_t func_code;
  uint16_t reg;
  uint8_t bytes; 
} request_id_t;

#define request_id_n 3
request_id_t request_ids[request_id_n] = {
  { 3, 0x1000, 2 },
  { 3, 0x1001, 2 },
  { 3, 0x102A, 2 }
};

typedef struct {
  uint8_t func_code;
  uint16_t reg_start;
  uint8_t reg_len;
  uint8_t bytes; 
} program_id_t;

#define program_id_n 6
program_id_t program_ids[program_id_n] = {
  { 3, 0x1030, 1, 2 }, //1st program
  { 3, 0x1040, 8, 2 }, //Last step
  { 3, 0x1050, 8, 2 }, //Additional cycles
  { 3, 0x1060, 8, 2 }, //Next pattern
  { 3, 0x2000, 64, 2 }, //SV
  { 3, 0x2080, 64, 2 }  //Time (minuts)
};

// 1st program, 8 last step, 8 additional cycles, 8 next pattern, 64 SV, 64 time = 153 data points.
uint16_t program[155];
uint16_t new_program[155];

char response[50];
int C_F = 0xFF;
uint8_t chunk = 0;
uint8_t set_program_flag = 0;
uint8_t replace_chart_data = 0;

char progress_status[2] = "";
char progress[500] = "";
uint16_t progress_len = 0;


//Called by obtain_time.
static void initialize_sntp(void)
{
    Serial.println("Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

//Gets NTP time.
static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while(timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        Serial.println("Waiting for system time to be set...");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}


//---Setup---
void setup() {
  
  //Serial.begin(115200);
  //delay(100);
  HardS.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //Put big data array in PSRAM.
  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
  if (psramInit()) { log_d("\nThe PSRAM is correctly initialized");
  } else { log_d("\nPSRAM does not work"); }
  
  int state_n_elements = ESP.getFreePsram() / (sizeof(state_t) * 2);
  state = (state_t *) ps_malloc(state_n_elements * sizeof(state_t));
  
  //Try to lower power req.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector  
  delay(500);

  // Connect to Wi-Fi network with SSID and password
  log_d("Connecting to %s", ssid);
  WiFi.begin(ssid, password);
  delay(500);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    log_d(".");
  }
  // Print local IP address and start web server
  log_d("WiFi connected.");
  log_d("IP address: %s",WiFi.localIP().toString().c_str());


  //Set time from NTP.
  time(&now);
  localtime_r(&now, &timeinfo);
  // Is time set? If not, tm_year will be (1970 - 1900).
  if (timeinfo.tm_year < (2016 - 1900)) {
    log_d("Time is not set yet. Connecting to WiFi and getting time over NTP.");
    obtain_time();
    // update 'now' variable with current time
    time(&now);
    start_time = now;
    prev_time = now;
   }

  // Set timezone to Eastern Standard Time and print local time
  setenv("TZ", "CST6CDT,M3.2.0/2,M11.1.0", 1);
  tzset();
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  tz_offset_secs = mktime(gmtime(&now)) - now;
  //log_d("tz_offset_secs: %d",tz_offset_secs);
  log_d("The current date/time in Chicago is: %s, gmt offset secs %d",strftime_buf,tz_offset_secs);

  //Request program state from controller.
  // 1st program, 8 last step, 8 additional cycles, 8 next pattern, 64 SV, 64 time = 153 data points.
  log_d("Request program state from controller (153 values).");
  uint8_t p = 0;
  uint8_t bytes = 0;
  char response[200];

  for (uint8_t i=0; i < program_id_n; i++) {
    uint8_t request_total = 0;
    uint16_t request_n = 0;
    
    for (uint16_t reg = program_ids[i].reg_start; reg < (program_ids[i].reg_start + program_ids[i].reg_len); reg += request_n) {
      request_n = program_ids[i].reg_len;
      if (request_n + request_total > program_ids[i].reg_len) { request_n = program_ids[i].reg_len - request_total; }
      if (request_n > REGISTERS_MAX_LIMIT) { request_n = REGISTERS_MAX_LIMIT; }
      request_total += request_n;
      
      lrc(SLAVE_ID,program_ids[i].func_code,reg,request_n,req_str);
      bytes = HardS.print(req_str);
      log_d("Request %s",req_str);
      delay(100);
      uint8_t b=0;
      while (HardS.available()) {
        response[b] = HardS.read();
        delay(10);
        b++;
      }
      response[b] = '\0';
      log_d("Response %s",response);
      //Serial.print(".");
      
      char c[3];
      uint8_t data[2] = { 0, 0 };
      uint16_t data_final;
      char out[100];
      uint8_t off;
      if (response[5] == '8' & response[6] == '2') {
        log_d("Error response.");
      } else {
        for (uint8_t k=0; k < request_n; k++) { //Response register.
          for (uint8_t j=0; j < program_ids[i].bytes; j++) { //Bytes (1 or 2) for each register.
            off = 7+(k*program_ids[i].bytes*2)+(j*2);
            strncpy(c,response+7+(k*program_ids[i].bytes*2)+(j*2),2);
            c[2] = '\0';
            data[j] = (uint8_t)strtol(c, NULL, 16);
          }
  
          if (program_ids[i].bytes == 1) { data_final = data[0]; }
          else { data_final = data[0]*256+data[1]; }
          //sprintf(out,"Hold bytes i,k %d,%d c %s off %d: %X%X -> %d  %s",i,k,c,off,data[0],data[1],data[0]*256+data[1],response);
          //Serial.println(out);
    
          //Save data.
          program[p] = data_final;
          p++;
        }
      }
    }
  }
  log_d("\nProgram data loaded");

  /*
  log_d("Program start: %d",program[0]); 
  for (int i=1; i < 154; i++) {
    log_d("Program %d: %d",i,program[i]);
    if ((i % 8) == 0) { log_d("---"); }
  }
  */

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    chunk = 0;
    AsyncWebServerResponse* response = request->beginChunkedResponse("text/html",
                                       [](uint8_t* buffer, size_t maxLen, size_t index) {
      size_t len = 0;
      log_d("Chunk: %d, maxlen %d",chunk,maxLen);
      //maxLen = maxLen >> 1;
      len = index_html(buffer,chunk);
      if (html_next == 0) { chunk++; }
      log_d(" Len: %d", len);
      return len;
    });

    state_sent_n = state_n; //Set a marker for last data sent for chart updates.
    response->addHeader("Server","Kiln Controller");
    request->send(response);

    replace_chart_data = 1;    
  });


  // XMLHttpRequest for chart data update.
  server.on("/chart", HTTP_GET, [](AsyncWebServerRequest *request){
    if(replace_chart_data == 1) {
      //log_d("Building JSON replace");
      //JSON response format:
      //'[{"series":"probe","update":0,"data":[[1612019322000,100],[1612020322000,200],[1612021322000,300]]}]'
      char chart_data[500] = "[{\"series\":\"program\",\"update\":0,\"data\":[";
      char program_data[400];
      print_kiln_program_table((uint8_t*)program_data);
      strcat(chart_data,program_data);
      strcat(chart_data,"]},{\"series\":\"probe\",\"update\":0,\"data\":[");
      
      char line[30];
      char comma[] = ",";
      for (int i=state_sent_n; i < state_n;i++) {
        if (i+1 == state_n) { comma[0] = '\0'; }      
        sprintf(line,"[%.f,%.1f]%s",((double)state[i].timestamp - tz_offset_secs)*1000.0,(double)state[i].pv/10.0,comma);
        //log_d("Line: %s comma: %s  data: %s",line,comma,chart_data);
        strcat(chart_data,line);
      }
      strcat(chart_data,"]}]");

      //log_d("Sent /chart response state_sent_n,_n %d,%d  %s",state_sent_n,state_n,chart_data);
      state_sent_n = state_n; //Move pointer to end of sent data.
     
      request->send(200, "application/json", chart_data); // Send new data points.
  
      replace_chart_data = 0;
      
    }else if (state_sent_n >= state_n - 1) {
      request->send(200, "application/json", "{}");
    } else {
      log_d("Building JSON update");
      //JSON response format:
      //'[{"series":"probe","update":1,"data":[[1612019322000,100],[1612020322000,200],[1612021322000,300]]}]'
      char chart_data[500] = "[{\"series\":\"probe\",\"update\":1,\"data\":[";
      char line[30];
      char comma[] = ",";
      for (int i=state_sent_n; i < state_n;i++) {
        if (i+1 == state_n) { comma[0] = '\0'; }      
        sprintf(line,"[%.f,%.1f]%s",((double)state[i].timestamp - tz_offset_secs)*1000.0,(double)state[i].pv/10.0,comma);
        //log_d("Line: %s comma: %s  data: %s",line,comma,chart_data);
        strcat(chart_data,line);
      }
      strcat(chart_data,"]}]");

      log_d("Sent /chart response update state_sent_n,_n %d,%d  %s",state_sent_n,state_n,chart_data);
      state_sent_n = state_n; //Move pointer to end of sent data.
     
      request->send(200, "application/json", chart_data); // Send new data points.
      /*
      for (int i=0; i < state_n;i++) {
        log_d("State %d: %f  %f",i,((double)state[i].timestamp - tz_offset_secs)*1000.0,(double)state[i].pv);
      }
      */
    }
  });


  
  // Send a GET request to <ESP_IP>/program?output=<inputMessage1>&state=<inputMessage2>
  server.on("/run", HTTP_POST, [] (AsyncWebServerRequest *request) {
  
    //Go to program step previous / next.
    if (request->hasParam("action", true)) {
      log_d("POST run: %s  set_program_flag: %d", request->getParam("action", true)->value(),set_program_flag);
      
      if (request->getParam("action", true)->value() == "prev") {
        set_program_flag = 3;
        log_d("POST prev set_program_flag: %d", set_program_flag);
        request->send(200, "text/plain", "OK"); // Progress holds state.
        return;
      } else if (request->getParam("action", true)->value() == "next") { 
        set_program_flag = 4;
        log_d("POST next set_program_flag: %d", set_program_flag);
        request->send(200, "text/plain", "OK"); // Progress holds state.
        return;
      } else if (request->getParam("action", true)->value() == "run") { //Pause/Run.
        if (set_program_flag == 0) { set_program_flag = 5; }
        else if (set_program_flag == 5) { set_program_flag = 6; }
        log_d("POST run set_program_flag: %d", set_program_flag);
        request->send(200, "text/plain", "OK"); // Progress holds state.
        return;
      }
    }
  });
  
  // Send a GET request to <ESP_IP>/program?output=<inputMessage1>&state=<inputMessage2>
  server.on("/program", HTTP_POST, [] (AsyncWebServerRequest *request) {

    // POST input values on <ESP_IP>/program?sv1=400.0&time1=60&sv2=...
    if (request->hasParam("start_pattern", true)) { new_program[0] = (uint16_t)request->getParam("start_pattern", true)->value().toInt(); }
    log_d("POST start_pattern: %d", new_program[0]);
    char param[16];
    for (uint8_t i=0; i<8; i++) {
      for (uint8_t j=0; j<8; j++) {
        sprintf(param,"sv%d%d",i,j);
        if (request->hasParam(param, true)) { new_program[j + i*8 + 25] = (uint16_t)(request->getParam(param, true)->value().toFloat() * 10); } //123.4 float
        log_d("POST i,j %d,%d  %s: %d  getParam %.1f -> %d", i, j, param, new_program[j + i*8 + 25],request->getParam(param, true)->value().toFloat(),(uint16_t)(request->getParam(param, true)->value().toFloat() * 10));
        sprintf(param,"time%d%d",i,j);
        if (request->hasParam(param, true)) { new_program[j + i*8 + 89] = (uint16_t)request->getParam(param, true)->value().toInt(); }
        log_d("POST i,j %d,%d  %s: %d", i,j,param, new_program[j + i*8 + 89]);
      }
      sprintf(param,"last_step%d",i);
      if (request->hasParam(param, true)) { new_program[i + 1] = (uint16_t)(request->getParam(param, true)->value().toInt()); }
      sprintf(param,"cycles%d",i);
      if (request->hasParam(param, true)) { new_program[i + 9] = (uint16_t)(request->getParam(param, true)->value().toInt()); }
      sprintf(param,"next%d",i);
      if (request->hasParam(param, true)) { new_program[i +17] = (uint16_t)(request->getParam(param, true)->value().toInt()); }
    }
      
    //len += sprintf(((char *)buffer + len), "<!DOCTYPE html><html>");
    //len += sprintf(((char *)buffer + len), "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    //len += sprintf(((char *)buffer + len), "<link rel=\"icon\" href=\"data:,\">");
    //len += sprintf(((char *)buffer + len), "</head><body><h1>Update Kiln Controller Program</h1>Stopping current program.<br>Writing data to controller.");
    //log_d("Async0 chunk %d, len %d",chunk,len);
    
    set_program_flag = 1; 
    strcpy(progress,"Updating controller program"); 
    
    request->send(200, "text/plain", "OK"); // Progress holds state.
    log_d("Sent /program response");
    //request->send(response);     
  });

  // Send a GET request to <ESP_IP>/progress
  server.on("/progress", HTTP_GET, [] (AsyncWebServerRequest *request) {
    char progress_data[2000] = "[";
    
    uint16_t get_n = state_n - 1;
    if (request->hasParam("t")) { get_n = (uint16_t)(request->getParam("t")->value().toInt()); }
    
    //log_d("temp_log range %d,%d",get_n,state_n);
    if (get_n + 1 < state_n-1) { 
      char temp_log[1500] = "\0";
      print_kiln_state(temp_log,get_n);
      //log_d("temp_log %s",temp_log);
      strcat(progress_data,temp_log);
      strcat(progress_data,",");
    }
    

    if (set_program_flag == 1) { strcat(progress,"."); }
    strcat(progress_data,"{\"jsid\":\"progress\",\"data\":\"");
    strcat(progress_data,progress);
    strcat(progress_data,"\"}]");
  
    request->send(200, "application/json", progress_data);
    if (set_program_flag == 2) { set_program_flag = 0; }; 
  });

  log_d("Server begin.");
  server.begin();
  log_d("Setup done.");
}


//Encode request or single write (holding or coil) MODBUS ASCII string.
//For request, bytes1/bytes2 are number of registers requested.
//For write, bytes1/bytes2 are the value to be written.
void lrc(uint8_t id, uint16_t req, uint16_t reg, uint16_t bytes,char *req_str) {
  uint8_t lrc;
  uint16_t total = id + req + (reg & 0xFF) + ((reg>>8) & 0xFF) + (bytes & 0xFF) + ((bytes>>8) & 0xFF); //Extract bytes from reg.
  
  while (total > 256) { total -= 256; }
  lrc = 256 - total;
 
  sprintf(req_str,":%02X%02X%04X%04X%02X\r\n",id,req,reg,bytes,lrc);
}


//Encode write multiple MODBUS ASCII string.
void set_lrc(uint8_t id, uint16_t reg, uint8_t registers_n, uint8_t p, uint16_t *program, char *req_str) {
  uint8_t lrc;
  char send_data[REGISTERS_MAX_LIMIT * 4 + 2];
  uint8_t data_n = 0;
  
  uint8_t req = WRITE_MULTIPLE_REGISTERS;
  
  uint16_t total = id + req + (reg & 0xFF) + ((reg>>8) & 0xFF) + registers_n + registers_n*2; //Extract bytes from reg.
  //log_d("set_lrc sum %d, %d, %d, %d, %d: total",id, req, (reg & 0xFF), ((reg>>8) & 0xFF), registers_n, total);

  for (uint8_t i = p; i < p + registers_n; i++) {
    total += ((program[i]>>8) & 0xFF) + (program[i] & 0xFF);
    sprintf(send_data + data_n,"%02X%02X",((program[i]>>8) & 0xFF),(program[i] & 0xFF));
    //log_d("set_lrc  program[i] %d  send_data %s",program[i],send_data);
    data_n += 4; //2 bytes = 4 ASCII char.
  }
  send_data[data_n] = '\0';

  //log_d("set_lrc data,data_n %s,%d total %d",send_data,data_n,total);
  
  while (total > 256) { total -= 256; }
  lrc = 256 - total;
 
  sprintf(req_str,":%02X%02X%04X%04X%02X%s%02X\r\n",id,req,reg,registers_n,registers_n*2,send_data,lrc); 
}

  
void loop(){

  //Serial.println("Looping.");
  //delay(100);
  
  time(&now);
  if (now > prev_time+CHECK_TEMP_EVERY_SECS) {
    //log_d("Call check_state %u prev %u",(unsigned)now,(unsigned)prev_time);  
    prev_time = now; 
    check_state();
    if (set_program_flag == 0) { status_html(); }
  }
  
  if (set_program_flag == 1) { set_program(); set_program_flag = 2; strcat(progress," programming complete."); }
  else if (set_program_flag == 3) { set_step(-1); set_program_flag = 0; sprintf(progress, "<div class='divc3 r big'>Restarting previous step.</div>"); }
  else if (set_program_flag == 4) { set_step(1); set_program_flag = 0; sprintf(progress, "<div class='divc3 r big'>Advanced to next step.</div>"); }
  else if (set_program_flag == 5) { set_pause(1); sprintf(progress, "<div class='divc3 r big'>Paused.</div>"); }
  else if (set_program_flag == 6) { set_pause(0); }
}


//Not implemented yet.
void set_step(uint8_t step_change) {
  return;
}


//Builds web page in chunks of < maxLen bytes, assumed to by ~5,000.  Try to keep chunks to < 4k bytes.
//Two varaibles control the packets of data: chunk and html_next.  'chunk' can get called repeatedly, 
//html_next tracks which rows have been sent for a chunk and == 0 if next chunk can proceed.
//
size_t index_html(uint8_t* buffer, uint8_t chunk) {
  uint16_t len = 0;
  if (chunk == 0) { 
  // Display the HTML web page
  len += sprintf(((char *)buffer + len), "<!DOCTYPE html><html>");
  len += sprintf(((char *)buffer + len), "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  len += sprintf(((char *)buffer + len), "<link rel=\"icon\" href=\"data:,\">");
  len += sprintf(((char *)buffer + len), "<script src='https://code.highcharts.com/highcharts.js'></script>");
  len += sprintf(((char *)buffer + len), "<script>var logn=0;function LogHide(){var nav=document.getElementById('temp_log');if(nav.style.height=='100%%'){nav.style.height='0';nav.style.opacity=0;}else{nav.style.height='100%%';nav.style.opacity=1;}};var xcall=setInterval(function(){getData();},2000);function getData(){var xhttp=new XMLHttpRequest();xhttp.onreadystatechange=function(){if(this.readyState==4&&this.status==200){var json=this.response;for(var i=0; i<json.length; i++){if(json[i].jsid=='logn'){logn=+(json[i].logn);}else if(json[i].jsid=='progress'){document.getElementById(json[i].jsid).innerHTML=json[i].data;}else{document.getElementById(json[i].jsid).innerHTML+=json[i].data;}}}};xhttp.open('GET','progress?t='+logn,true);xhttp.responseType='json';xhttp.send();}</script>");
  len += sprintf(((char *)buffer + len), "<script>function getChart(series){var xhttp2=new XMLHttpRequest();xhttp2.onreadystatechange=function(){if(this.readyState==4&&this.status==200){var json=this.response;for(var i=0;i<json.length;i++){var s=0;if(json[i].series=='probe'){s=1;}if(json[i].update==1){for(var j=0;j<json[i].data.length;j++){x=+(json[i].data[j][0]);y=+(json[i].data[j][1]);series[s].addPoint([Math.round(x),y],true,false);}}else{document.getElementById('temp_log').innerHTML='';logn=0;var newdata=[];for(var j=0;j<json[i].data.length;j++){x=+(json[i].data[j][0]);y=+(json[i].data[j][1]);newdata.push([Math.round(x),y]);}series[s].setData(newdata,true);}}}};xhttp2.open('GET','/chart',true);xhttp2.responseType='json';xhttp2.send();};</script>");
  len += sprintf(((char *)buffer + len), "<script>function submitForm(F){var xhr=new XMLHttpRequest();xhr.open(F.method,F.getAttribute('action'));xhr.send(new FormData(F));return false;}</script>");
  return len;
  }

  else if (chunk == 1) {
    len += sprintf(((char *)buffer + len), "<style>html{font-family:Helvetica;display:inline-block;margin:0px auto;text-align:center;}h2{font-family:Arial;font-size:1.5rem;text-align:center;}td,th{text-align:center;}div{border:.03em solid black;}.divp{display:flex;}.divc{flex:1;}.divc2{flex:0 1 40px;}.divc3{flex:0 1 150px;}.divl{border:none;}.g{background-color:green;}.hider{transition: opactity 3s ease-in;opacity: 0;cursor: pointer;height:0;overflow:auto;}.r{background-color:#CD5C5C;}.b{background-color:tan;}.gr{background-color:#eee;}.but{height:50%%;padding:0 2px 0 2px;}.big{font-size:1.25rem;text-align:center;line-height:2rem;}</style>");
    len += sprintf(((char *)buffer + len), "</head><body><h2>Kiln Controller</h2><div id='progress' class='divp gr'>%s</div><div id='chart-temperature' class='container'></div>",progress);
    return len;
  }
  else if (chunk == 2) {
      // CSS to style 
    len = print_kiln_program(buffer, len);
    //len += sprintf(((char *)buffer + len), "<br>");
    return len;
  }
  else if (chunk == 3) { 
    len = print_kiln_state_head(buffer);
    if (html_next == 0) {
      len += sprintf(((char *)buffer + len), "</body>");
      // Web Page Heading
      len += sprintf(((char *)buffer + len), "<script>new Highcharts.chart('chart-temperature',{chart:{events:{load:function(){var series=this.series;setInterval(function(){getChart(series)},10000);}}},title:{text:'Kiln Temperature'},colors:['#39F','#000'],legend:{layout:'vertical',align:'right',verticalAlign:'top',floating:true,y:25,x:-10},series:[{showInLegend:true,name:'Program',data:[");
    }
    return len;
  }
  else if (chunk == 4) { 
    len = print_kiln_program_table(buffer);
    if (html_next == 0) {
      len += sprintf(((char *)buffer + len), "]},{showInLegend:true,name:'Run Data',data:[");
    }
    return len;
  }
  else if (chunk == 5) { 
    len = print_kiln_state_table(buffer,0);
    if (html_next == 0) {
        String C_F;
        if (state[0].leds & 4) { C_F = "F"; } else { C_F = "C"; }
      //len += sprintf(((char *)buffer + len), "]}],plotOptions:{line:{animation:false,dataLabels:{enabled:true}}},xAxis:{title:{text:'Time (HH:MM:SS)'},type:'datetime',");
      //len += sprintf(((char *)buffer + len), "dateTimeLabelFormats:{millisecond:'%%H:%%M:%%S',second:'%%H:%%M:%%S',minute:'%%H:%%M',hour:'%%H:%%M'}},yAxis:{title:{text:'Temperature'}},credits:{enabled:false}});</script>");
      len += sprintf(((char *)buffer + len), "]}],tooltip:{formatter: function() {return '<b>'+this.series.name +'</b><br/>'+Highcharts.dateFormat('%%e %%b %%Y %%H:%%M:%%S',new Date(this.x))+'<br/>'+this.y+'\\u00B0'+'%s';}},xAxis:[{title:{text:'Run Schedule (HH:MM:SS)'},labels:{formatter:function(){return Highcharts.dateFormat('%%H:%%M:%%S',this.value);}}}],plotOptions:{line:{animation:false,dataLabels:{enabled:true}}},",C_F);
      len += sprintf(((char *)buffer + len), "yAxis:{title:{text:'Temperature'}},credits:{enabled:false}});</script>");

      len += sprintf(((char *)buffer + len), "</html>");
    }
    return len;
  }
  else { return len; }
}


//Form status HTML.
void status_html(void) {
    
  time(&now);
  //log_d("Time delta: %d, %d delta %d",now,state_current.run_start_time,now-state_current.run_start_time);
    localtime_r(&state_current.run_start_time, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);
  if (state_current.run_stop == 1) { sprintf(progress, "<div class='divc3 g big'>Start %s<br>Run %02d:%02d:%02d</div>",strftime_buf,(now-state_current.run_start_time) / (60*60),((now-state_current.run_start_time) / 60) % 60,(now-state_current.run_start_time) % 60); }
  else { sprintf(progress, "<div class='divc3 r big'>Stopped</div>"); }

  uint16_t s = 0;
  if (state_n > 0) { s = state_n-1; }
 
  String C_F;
  if (state[s].leds & 4) { C_F = "F"; } else { C_F = "C"; }
  sprintf(progress + strlen(progress), "<div class='divc'><div>%.1f&#176;%s Current</div><div>%.1f&#176;%s Set</div></div><div class='divc2'>&nbsp;",state[state_n-1].pv/10.0,C_F,state[state_n-1].sv/10.0,C_F);

  //log_d("State leds: %d",state[s].leds);
  uint8_t buttons = 1;
  if (state[s].leds & 64) { sprintf(progress + strlen(progress), "<div class='g but'>OUT1</div>"); } 
  else { sprintf(progress + strlen(progress), "<div class='r but'>OUT1</div>"); }
  if (state[s].leds & 32) { sprintf(progress + strlen(progress), "<div class='g but'>OUT2</div>"); buttons++; }
  if (buttons == 2) { buttons=0; sprintf(progress + strlen(progress), "</div><div class='divc2'>"); }
  if (state[s].leds & 128) { sprintf(progress + strlen(progress), "<div class='g but'>AT</div>"); buttons++; }
  if (buttons == 2) { buttons=0; sprintf(progress + strlen(progress), "</div><div class='divc2'>"); }
  if (state[s].leds & 16) { sprintf(progress + strlen(progress), "<div class='g but'>ALM1</div>"); buttons++; }
  if (buttons == 2) { buttons=0; sprintf(progress + strlen(progress), "</div><div class='divc2'>"); }
  if (state[s].leds & 2) { sprintf(progress + strlen(progress), "<div class='g but'>ALM2</div>"); buttons++; }
  if (buttons == 2) { buttons=0; sprintf(progress + strlen(progress), "</div><div class='divc2'>"); }
  if (state[s].leds & 1) { sprintf(progress + strlen(progress), "<div class='g but'>ALM3</div>"); buttons++; }
  sprintf(progress + strlen(progress), "</div><div class='divc'>");

  switch(state_current.control_mode) {
    case 0: sprintf(progress + strlen(progress), "<div>Mode: PID</div>"); break;
    case 1: sprintf(progress + strlen(progress), "<div>Mode: On/Off</div>"); break;
    case 2: sprintf(progress + strlen(progress), "<div>Mode: Manual</div>"); break;
    case 3: sprintf(progress + strlen(progress), "<div>Mode: Ramp/Soak</div>"); break;
  }
  
  log_d("probe_type: %d",state_current.probe_type);
  
  switch(state_current.probe_type) { 
    case 0: sprintf(progress + strlen(progress), "<div>Temp probe type: K</div>"); break;
    case 1: sprintf(progress + strlen(progress), "<div>Temp probe type: J</div>"); break;
    case 2: sprintf(progress + strlen(progress), "<div>Temp probe type: T</div>"); break;
    case 3: sprintf(progress + strlen(progress), "<div>Temp probe type: E</div>"); break;
    case 4: sprintf(progress + strlen(progress), "<div>Temp probe type: N</div>"); break;
    case 5: sprintf(progress + strlen(progress), "<div>Temp probe type: R</div>"); break;
    case 6: sprintf(progress + strlen(progress), "<div>Temp probe type: S</div>"); break;
    default: sprintf(progress + strlen(progress), "<div>Temp probe type: not set</div>");
  }

  sprintf(progress + strlen(progress), "</div></div>\0");
}


//Add HTML stub to display temp and status display.
size_t print_kiln_state_head(uint8_t* buffer) {
  uint16_t len = 0;
  if (html_next == 0) {
    len += sprintf(((char *)buffer + len), "<div onclick='LogHide()' class='divl'><h3>Temperature Log (click to toggle)</h3><div class='hider' id='temp_log'></div></div>");
  }
  
  return len;
}


//Add HTML to display temp and status display.
//Bit 0 = ALM3 
//Bit 1 = ALM2 
//Bit 2 = °F
//Bit 3 = °C
//Bit 4 = ALM1
//Bit 5 = OUT2
//Bit 6 = OUT1
//Bit 7 = AT
void print_kiln_state(char *buf, uint16_t get_n) {
 
  strcat(buf, "{\"jsid\":\"temp_log\",\"data\":\"");

  uint16_t logn = get_n;
  char temp[120];
  for (uint16_t i = get_n+1; i < state_n-1 & i < (get_n + 10); i++) {
    String C_F;
    if (state[i].leds & 4) { C_F = "F"; }
    else { C_F = "C"; }

    String leds = "";
    if (state[i].leds & 64) { leds += "OUT1 "; }
    if (state[i].leds & 32) { leds += "OUT2 "; }
    if (state[i].leds & 128) { leds += "AT "; }
    if (state[i].leds & 16) { leds += "ALM1 "; }
    if (state[i].leds & 2) { leds += "ALM2 "; }
    if (state[i].leds & 1) { leds += "ALM3 "; }
    localtime_r(&state[i].timestamp, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%y/%m/%d %H:%M:%S", &timeinfo);
    sprintf(temp, "%s&nbsp;&nbsp;PV: %.1f&#176;%s SV: %.1f&#176;%s LEDs: %s<br>",strftime_buf,state[i].pv/10.0,C_F,state[i].sv/10.0,C_F,leds);
    strcat(buf,temp);
    logn = i;
    //log_d("print_kiln_state tmp %s",temp);
  }
  //log_d("print_kiln_state buf %s",buf);
  //String temp2 = "},{\"jsid\":\"logn\",\"logn\":\"" + String(logn) + "\"}";
  sprintf(temp, "\"},{\"jsid\":\"logn\",\"logn\":%d}",logn);
  //log_d("print_kiln_state temp2 %s",temp);
  strcat(buf, temp);
  //log_d("print_kiln_state buf2 %s",buf);
  //strcat(buffer,temp);
}


//Add HTML for chart data to display temp values.
size_t print_kiln_state_table(uint8_t* buffer, uint8_t type) {
  uint16_t len = 0;
  uint16_t i = html_next;
  for (; i < state_n & i < html_next + 20; i++) {
    if (type == 0) { len += sprintf(((char *)buffer + len), "[%.f, %.1f],",((double)state[i].timestamp - tz_offset_secs)*1000.0,state[i].pv/10.0); }
    else { len += sprintf(((char *)buffer + len), "[%.f, %.1f],",((double)state[i].timestamp - tz_offset_secs)*1000.0,state[i].sv/10.0); }
    log_d("State: [%.f, %.1f],",((double)state[i].timestamp - tz_offset_secs)*1000.0,state[i].pv/10.0);
  }
  
  if (i == state_n) { html_next = 0; } 
  else { html_next = i; }
  
  return len;
}


//Add HTML for chart data to display current program.
// 1st program, 8 last step, 8 additional cycles, 8 next pattern, 64 SV, 64 time = 153 data points.
size_t print_kiln_program_table(uint8_t* buffer) {
  uint16_t len = 0;
  uint16_t i = html_next;
  
  uint8_t prog = program[0];
  uint16_t n = 0;
  uint8_t last_step = program[prog + 1];
  int8_t cycles = program[prog + 9];
  uint8_t pstep = 0;
  char line[25];
  time_t time_p = state_current.run_start_time;

  //Start time point.
  //localtime_r(&time_p, &timeinfo);
  //strftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);
  if (html_next == 0) { 
    len += sprintf(((char *)buffer + len), "[%.f, %.1f],",((double)time_p - tz_offset_secs)*1000.0, state[0].pv/10.0);
    //log_d("Program start: [%.f, %.1f],",((double)time_p - tz_offset_secs)*1000.0, state[0].pv/10.0);
  }
  while (n < 1000) {
    char line2[100];
    //log_d("n: %d prog: %d pstep: %d last_step: %d cycles: %d  time_p: %d",n,prog,pstep,last_step,cycles,time_p);

    time_p += program[pstep + prog*8 + 89] * 60; //Time in seconds.
    //localtime_r(&time_p, &timeinfo);
    //trftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);
    len += sprintf(((char *)buffer + len), "[%.f, %.1f],",((double)time_p - tz_offset_secs)*1000.0, program[pstep + prog*8 + 25]/10.0);
    //log_d("Program: [%.f, %.1f],",((double)time_p - tz_offset_secs)*1000.0, program[pstep + prog*8 + 25]/10.0);
    
    n++;
    pstep++;
    if (pstep > last_step) {
      cycles--;
      if (cycles < 0) {
        if (prog == program[prog + 17]) { break; } //Same prog cycles infinitely, so only show once.
        else { prog = program[prog + 17]; }
        last_step = program[prog + 1];
        cycles = program[prog + 9];
        pstep = 0;
        //log_d("New prog: %d  last %d  cycl %d",prog,last_step,cycles);
      } 
      else { pstep = 0; }
    }
  
    if (prog == 8) { break; }  
  }
  len -= 1;
  buffer[len] = '\0';
  return len;
}


//Add HTML for kiln programming buttons.
size_t print_kiln_program (uint8_t* buffer, uint16_t len) {
  uint8_t start = 0;
  if (html_next == 0) {
    len += sprintf(((char *)buffer + len), "<form action='/run' method='post' onsubmit='return submitForm(this);'><input type='hidden' name='action' value=''><input type='submit' name='step_prev' value='< Prev step' onclick=\"this.form.elements['action'].value='prev';\"/><input type='submit' name='run' value='Run/Pause' onclick=\"this.form.elements['action'].value='run';\"/><input type='submit' name='step_next' value='Next step >' onclick=\"this.form.elements['action'].value='next';\"/></form>");
    len += sprintf(((char *)buffer + len), "<h3>Ramp/Soak Program</h3><form action='/program' method='post' onsubmit='return submitForm(this);'>Starting Ramp / Soak Pattern <input type='text' name='start_pattern' size='1' value='%d'/><table><thead><tr><td>&nbsp;</td><td>SV 0</td><td>Time 0</td><td>SV 1</td><td>Time 1</td><td>SV 2</td><td>Time 2</td><td>SV 3</td><td>Time 3</td><td>SV 4</td><td>Time 4</td><td>SV 5</td><td>Time 5</td><td>SV 6</td><td>Time 6</td><td>SV 7</td><td>Time 7</td><td>Last Step</td><td>+Cycles</td><td>Next Pattern</td></tr></thead>",program[0]);
  }
  
  uint16_t i = html_next;
  //Print the 8 Ramp/Soak patterns, one on each row.
  for (; i < (html_next+1); i++) {
    len += sprintf(((char *)buffer + len), "<tr><th>Pattern %d</th>",i);
    //Serial.println((char *)buffer);
    
    //SV / time.
    for (uint8_t j=0; j<8; j++) {
      len += sprintf(((char *)buffer + len), "<td><input type='text' name='sv%d%d' size='6' value='%.1f'/></td>",i,j,program[j + i*8 + 25]/10.0);
      //Serial.println((char *)buffer); 

      len += sprintf(((char *)buffer + len), "<td><input type='text' name='time%d%d' size='4' value='%d'/></td>",i,j,program[j + i*8 + 89]);
      //Serial.println((char *)buffer); 
    }
    //Last step.
    len += sprintf(((char *)buffer + len), "<td><input type='text' name='last_step%d' size='1' value='%d'/></td>",i,program[i + 1]);
    //Serial.println((char *)buffer); 

    //Additional cycles
    len += sprintf(((char *)buffer + len), "<td><input type='text' name='cycles%d' size='1' value='%d'/></td>",i,program[i + 9]);
    //Serial.println((char *)buffer);  
    
    //Next pattern.
    len += sprintf(((char *)buffer + len), "<td><input type='text' name='next%d' size='1' value='%d'/></td></tr>",i,program[i + 17]);
    //Serial.println((char *)buffer); 
  }

  if (i == 8) { html_next = 0; } 
  else { html_next = i; }

  if (html_next == 0) { len += sprintf(((char *)buffer + len), "</table><br><input type='submit' value='Program controller'></form>"); }
  return len;
}


//Write new Ramp/Soak program to the controller.
//
//1) Set Run/Stop (0x0814) to stop.
//2) If needed, set Input Range High & Low to values in the new program.
//3) Update program values.
//4) Start run immediately if requested.
//
void set_program(void) {
  uint8_t p = 0;
  uint8_t bytes = 0;
  char current_req_str[60];
  char line[50];
  uint16_t P2_1_RunStop = 0x0814;
  
  //Stop running program.
  lrc(SLAVE_ID,WRITE_SINGLE_COIL,P2_1_RunStop,0x0000,req_str);
    
  update_min_max_temp(program);
    
  //Update program values.
  //html_next allows this to run within the Async web server process.  html_next gets advanced by 1 every call, set to 0 when done.
  for (uint8_t i=0; i < program_id_n; i++) {
    uint8_t request_total = 0;
    uint8_t request_n = 0;

    for (uint16_t reg = program_ids[i].reg_start; reg < (program_ids[i].reg_start + program_ids[i].reg_len); reg += (uint16_t)request_n) {
      request_n = program_ids[i].reg_len;
      if (request_n + request_total > program_ids[i].reg_len) { request_n = program_ids[i].reg_len - request_total; }
      if (request_n > REGISTERS_MAX_LIMIT) { request_n = REGISTERS_MAX_LIMIT; }
      request_total += request_n;

      log_d("Set: i,request_n,p %d, %d, %d  reg %X  reg_len %d",i,request_n,p,reg,program_ids[i].reg_len); 
           
      set_lrc(SLAVE_ID,reg,request_n,p,program,current_req_str);
      set_lrc(SLAVE_ID,reg,request_n,p,new_program,req_str);
      log_d("Pair:\n%s%s",current_req_str,req_str);
      
      p += request_n;
      
      if (strcmp(current_req_str,req_str) == 0) {
        log_d("No update");
        continue;
      }
    
      bytes = HardS.print(req_str);
      log_d("Request %s",req_str);
      delay(500);
      uint8_t b=0;
      while (HardS.available()) {
        response[b] = HardS.read();
        delay(20);
        b++;
      }
      response[b] = '\0';
      if (b >= 0) { log_d("Set response %d bytes: --%s--",b,response); }
      else { log_d("No response"); }
    }
  }

  log_d("Start program");
  //Start program.
  lrc(SLAVE_ID,WRITE_SINGLE_COIL,P2_1_RunStop,0xFF00,req_str);

  //Reset run start time.
  time(&now);
  state_current.run_start_time = now;
  state_n = 0;
  
  replace_chart_data = 1; //Flag for /chart update to replace existing data.
  log_d("End of set_program");

  //Update program state with new program.
  for (uint8_t i=1; i < 154; i++) { program[i] = new_program[i]; }
}



//Write new Ramp/Soak program to the controller.
//
//1) Set Run/Stop (0x0814) to stop.
//2) If needed, set Input Range High & Low to values in the new program.
//3) Update program values.
//4) Start run immediately if requested.
//
void set_pause(uint8_t state) {
  
  
  if (state == 1) { //Stop running program.
    modbus_data_single(WRITE_SINGLE_REGISTER, REGISTER_HOLD, 0x0001, RESPONSE_BYTES_COIL); 
    state_current.run_stop = 0; //Stop
  }
  else if (state == 0) {  //Restart
    modbus_data_single(WRITE_SINGLE_REGISTER, REGISTER_HOLD, 0x0000, RESPONSE_BYTES_COIL);
    state_current.run_stop = 1; //Run
    set_program_flag = 0;
  }
}


//Check current min/max temps, update to include range in new program values.
void update_min_max_temp(uint16_t *program) { 
  
  int16_t current_min_temp = (int16_t)modbus_data_single(READ_HOLDING_REGISTER,0x1003,0x0001,RESPONSE_BYTES_REGISTER);
  int16_t current_max_temp = (int16_t)modbus_data_single(READ_HOLDING_REGISTER,0x1002,0x0001,RESPONSE_BYTES_REGISTER);
  
  //Check if new program is outside the temp limits.
  int16_t min_temp = current_min_temp; int16_t max_temp = current_max_temp;
  for (uint8_t i=25; i < 25+64; i++) {
    if ((int16_t)program[i] < min_temp) {  min_temp = (int16_t)program[i];}
    if ((int16_t)program[i] > max_temp) {  max_temp = (int16_t)program[i];}
  }
  log_d("Program min,max temp %d,%d",min_temp,max_temp);

  //Update temp limits if needed.
  if (min_temp < current_min_temp) { log_d("Write min_temp"); modbus_data_single(WRITE_SINGLE_REGISTER,0x1003,(uint16_t)min_temp,RESPONSE_BYTES_REGISTER); }
  if (max_temp > current_max_temp) { log_d("Write max_temp"); modbus_data_single(WRITE_SINGLE_REGISTER,0x1002,(uint16_t)max_temp,RESPONSE_BYTES_REGISTER); }
}


//Make a request for modbus data, single register or coil.
uint16_t modbus_data_single (uint8_t func_code, uint16_t reg, uint16_t value, uint8_t response_bytes) {
  
  char response[50];
  
  lrc(SLAVE_ID,func_code,reg,value,req_str);
  uint8_t bytes = HardS.print(req_str);
  //log_d("Ask: %s",req_str);
  delay(75);  //Shorter delays give error.
  
  uint8_t b=0;
  while (HardS.available()) {
    response[b] = HardS.read();
    b++;
  }
  response[b] = '\0';
  
  //Serial.print("Resp: ");
  //Serial.println(response);
  
  //Parse response.
  char c[3];
  uint8_t data[2];
  uint16_t data_final;
  
  for (uint8_t j=0; j < response_bytes; j++) { 
    if (response[5] != '8') {
      strncpy(c,response+7+(j*2),2);
      c[3] = '\0';
      data[j] = (uint8_t)strtol(c, NULL, 16);
    }

    if (response_bytes == 1) { data_final = data[0]; }
    else { data_final = data[0]*256+data[1]; }
  }

  log_d("Hold bytes: %X%X -> %d  response %d bytes --%s--",data[0],data[1],data[0]*256+data[1],b,response);

  return data_final;
}


//Request registers defining current state.
//
void check_state (void) {
  uint16_t data_final;
  uint8_t response_bytes;
  time(&now);
  state[state_n].timestamp = now;
  
  for (uint8_t i=0; i < request_id_n; i++) {

    if (request_ids[i].reg >= 0x1000) { response_bytes = RESPONSE_BYTES_REGISTER; }
    else { response_bytes = RESPONSE_BYTES_COIL; }
    data_final = modbus_data_single(request_ids[i].func_code, request_ids[i].reg, 0x0001, response_bytes);
    log_d("check state case %d, value %d", i, data_final);

    //Save data.
    switch(i) {
      case 0:
        state[state_n].pv = data_final; break;
      case 1:
        state[state_n].sv = data_final; break;
      case 2:
        state[state_n].leds = data_final; break;
    }
  }
  //log_d("Check State %d: %u as f %f or %f, %d",state_n,(unsigned)state[state_n].timestamp,(double)state[state_n].timestamp,(double)state[state_n].timestamp,state[state_n].pv);
  
  state_n++;
  if (state_n == state_n_elements) { state_n = 0; } //Reset if this overflows the array.

  //Read but don't save these state values.
  if (state_current.probe_type > 17) { state_current.probe_type = modbus_data_single(READ_HOLDING_REGISTER, REGISTER_PROBE_TYPE, 0x0001, RESPONSE_BYTES_REGISTER); }
  if (state_current.run_start_time == 0) { state_current.run_start_time = now; }
  //log_d("Run_start_time %d",state_current.run_start_time);
  
  log_d("check_state Probe_type: %d",state_current.probe_type);

  state_current.control_mode = modbus_data_single(READ_HOLDING_REGISTER, REGISTER_CONTROL_MODE, 0x0001, RESPONSE_BYTES_REGISTER);
  state_current.run_stop = modbus_data_single(READ_COIL, REGISTER_RUN_STOP, 0x0001, RESPONSE_BYTES_COIL);
  state_current.hold = modbus_data_single(READ_COIL, REGISTER_HOLD, 0x0001, RESPONSE_BYTES_COIL);

  log_d("check_state--control_mode: %d, run_stop %d, hold %d",state_current.control_mode, state_current.run_stop, state_current.hold);
}
