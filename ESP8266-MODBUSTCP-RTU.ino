//
//我用的硬件是nodeMCU(ESP-12F)+RSM3485ECHT,板载CH340用作下载口和诊断，另外使用CP2102N的RS485数据线连接软串口用作modbus slave的测试端口。
//实测支持modbus poll多ID同时读modbus slave，但由于系统是同步读写数据故性能要差一些，每秒大概可以处理5次请求。
//手测支持一个ID同时读最多29个(03)寄存器,超出modbus poll报"transcation id error"，诊断窗口不停打印"TCP IP: 192.168.3.240 Fn: 03, len: 5"
//然后只能复位ESP8266重连才可以正常运行。
//

#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else //ESP32
 #include <WiFi.h>
#endif
#include <ModbusTCP.h>
#include <ModbusRTU.h>
#include <SoftwareSerial.h>
//SoftwareSerial S(13, 15);
EspSoftwareSerial::UART S;//这里用到EspSoftwareSerial的软件串口库，定义RX:D7(13),TX:D8(15),
//#include "StreamBuf.h"
//#define BSIZE 1024
//uint8_t buf1[BSIZE];
//uint8_t buf2[BSIZE];
//StreamBuf S1(buf1, BSIZE);
//StreamBuf S2(buf2, BSIZE);
//DuplexBuf P1(&S1, &S2);
//DuplexBuf P2(&S2, &S1);
//ModbusRTU sym;

int DE_RE = 2//RE在modbus库使用而不在串口库中，使用D4(2)

ModbusRTU rtu;
ModbusTCP tcp;

IPAddress srcIp;


uint16_t transRunning = 0;  // Currently executed ModbusTCP transaction
uint8_t slaveRunning = 0;   // Current request slave
 
bool cbTcpTrans(Modbus::ResultCode event, uint16_t transactionId, void* data) { // Modbus Transaction callback
  if (event != Modbus::EX_SUCCESS)                  // If transaction got an error
    Serial.printf("Modbus result: %02X, Mem: %d\n", event, ESP.getFreeHeap());  // Display Modbus error code (222527)
  if (event == Modbus::EX_TIMEOUT) {    // If Transaction timeout took place
    tcp.disconnect(tcp.eventSource());          // Close connection
  }
  return true;
}

bool cbRtuTrans(Modbus::ResultCode event, uint16_t transactionId, void* data) {
    if (event != Modbus::EX_SUCCESS)                  // If transaction got an error
      Serial.printf("Modbus result: %02X, Mem: %d\n", event, ESP.getFreeHeap());  // Display Modbus error code (222527)
    return true;
}


// Callback receives raw data 
Modbus::ResultCode cbTcpRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;
  
  Serial.print("TCP IP: ");
  Serial.print(IPAddress(src->ipaddr));
  Serial.printf(" Fn: %02X, len: %d \n\r", data[0], len);

  if (transRunning) { // Note that we can't process new requests from TCP-side while waiting for responce from RTU-side.
    tcp.errorResponce(src->ipaddr, (Modbus::FunctionCode)data[0], Modbus::EX_SLAVE_DEVICE_BUSY);
    return Modbus::EX_SLAVE_DEVICE_BUSY;
  }

  srcIp = src->ipaddr;
  
  slaveRunning = src->slaveId;
  
  transRunning = src->transactionId;
  
  rtu.rawRequest(slaveRunning, data, len, cbRtuTrans);

  return Modbus::EX_SUCCESS;  
  
}


// Callback receives raw data from ModbusTCP and sends it on behalf of slave (slaveRunning) to master
Modbus::ResultCode cbRtuRaw(uint8_t* data, uint8_t len, void* custom) {
  auto src = (Modbus::frame_arg_t*) custom;
  tcp.setTransactionId(transRunning); // Set transaction id as per incoming request
  uint16_t succeed = tcp.rawResponce(srcIp, data, len, slaveRunning);
  if (!succeed){
    Serial.print("fail");
  }
  Serial.printf("RTU Slave: %d, Fn: %02X, len: %d, ", src->slaveId, data[0], len);
  Serial.print("Response TCP IP: ");
  Serial.println(srcIp);
  
  transRunning = 0;
  slaveRunning = 0;
  return Modbus::EX_PASSTHROUGH;
}


void setup() {
  Serial.begin(115200);
  WiFi.begin("SSID", "PASSWORD");//这里根据自己情况改
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
    
  tcp.server(); // Initialize ModbusTCP to pracess as server
  tcp.onRaw(cbTcpRaw); // Assign raw data processing callback
  
  S.begin(9600, SWSERIAL_8N1, 13, 15 ,false);//串口速率9600,8数据位,1停止位,无校验
  rtu.begin(&S, DE_RE ,false);  // Specify RE_DE control pin，参数flase用来反转DE管脚的逻辑，我用的RSM3485ECHT要求低电平发送数据
  //sym.begin((Stream*)&P2);
  //sym.slave(1);
  //sym.addHreg(1, 100);
  //rtu.begin((Stream*)&P1);  // Specify RE_DE control pin
  rtu.master(); // Initialize ModbusRTU as master
  rtu.onRaw(cbRtuRaw); // Assign raw data processing callback
}

void loop() { 
  //sym.task();
  rtu.task();
  tcp.task();
  yield();
}
