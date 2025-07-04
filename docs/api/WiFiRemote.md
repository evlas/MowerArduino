[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)

# WiFiRemote Class

## Methods

### Quick Reference

- [void begin(unsigned long baudRate = SERIAL_WIFI_BAUD)](#begin)
- [uint16_t calculateCRC16(const uint8_t* data, uint16_t length)](#calculatecrc16)
- [void checkConnection()](#checkconnection)
- [void createStatusJson(JsonDocument& json)](#createstatusjson)
- [void getRemoteStatus(RemoteStatus& status)](#getremotestatus)
- [void handleEmergencyCommand()](#handleemergencycommand)
- [void handleJsonCommand(const String& jsonStr)](#handlejsoncommand)
- [void handleMoveCommand(const uint8_t* data, uint16_t length)](#handlemovecommand)
- [bool isConnected()](#isconnected)
- [bool isRemoteControlEnabled()](#isremotecontrolenabled)
- [void onConnectionStateChange(ConnectionStateCallback callback)](#onconnectionstatechange)
- [void processIncomingData()](#processincomingdata)
- [void processPacket(const uint8_t* data, uint16_t length)](#processpacket)
- [bool processRemoteCommand(RemoteCommandType cmd, const CommandData& data = {})](#processremotecommand)
- [bool registerCommand(const char* command, JsonCommandCallback callback)](#registercommand)
- [void resetConnection()](#resetconnection)
- [void sendAck(MessageType cmdType)](#sendack)
- [void sendError(uint8_t errorCode, const String& message = "")](#senderror)
- [void sendHeartbeat()](#sendheartbeat)
- [void sendJsonResponse(const JsonDocument& json)](#sendjsonresponse)
- [void sendLog(const String& message, bool isError = false)](#sendlog)
- [bool sendPacket(MessageType type, const void* data = nullptr, uint16_t length = 0)](#sendpacket)
- [void sendRobotStatus()](#sendrobotstatus)
- [void sendStatus(bool force = false)](#sendstatus)
- [void setConnectionState(ConnectionState state)](#setconnectionstate)
- [void setRemoteControlEnabled(bool enable)](#setremotecontrolenabled)
- [void unregisterCommand(const char* command)](#unregistercommand)
- [void update()](#update)

### Method Details

#### begin

```cpp
void begin(unsigned long baudRate = SERIAL_WIFI_BAUD)
```

**Parameters:**

- `SERIAL_WIFI_BAUD` (unsigned long baudRate =): No description

**Returns:**

void - 

---

#### calculateCRC16

```cpp
uint16_t calculateCRC16(const uint8_t* data, uint16_t length)
```

**Parameters:**

- `data` (const uint8_t*): No description
- `length` (uint16_t): No description

**Returns:**

uint16_t - 

---

#### checkConnection

```cpp
void checkConnection()
```

**Returns:**

void - 

---

#### createStatusJson

```cpp
void createStatusJson(JsonDocument& json)
```

**Parameters:**

- `json` (JsonDocument&): No description

**Returns:**

void - 

---

#### getRemoteStatus

```cpp
void getRemoteStatus(RemoteStatus& status)
```

**Parameters:**

- `status` (RemoteStatus&): No description

**Returns:**

void - 

---

#### handleEmergencyCommand

```cpp
void handleEmergencyCommand()
```

**Returns:**

void - 

---

#### handleJsonCommand

```cpp
void handleJsonCommand(const String& jsonStr)
```

**Parameters:**

- `jsonStr` (const String&): No description

**Returns:**

void - 

---

#### handleMoveCommand

```cpp
void handleMoveCommand(const uint8_t* data, uint16_t length)
```

**Parameters:**

- `data` (const uint8_t*): No description
- `length` (uint16_t): No description

**Returns:**

void - 

---

#### isConnected

```cpp
bool isConnected()
```

**Returns:**

bool - 

---

#### isRemoteControlEnabled

```cpp
bool isRemoteControlEnabled()
```

**Returns:**

bool - 

---

#### onConnectionStateChange

```cpp
void onConnectionStateChange(ConnectionStateCallback callback)
```

**Parameters:**

- `callback` (ConnectionStateCallback): No description

**Returns:**

void - 

---

#### processIncomingData

```cpp
void processIncomingData()
```

**Returns:**

void - 

---

#### processPacket

```cpp
void processPacket(const uint8_t* data, uint16_t length)
```

**Parameters:**

- `data` (const uint8_t*): No description
- `length` (uint16_t): No description

**Returns:**

void - 

---

#### processRemoteCommand

```cpp
bool processRemoteCommand(RemoteCommandType cmd, const CommandData& data = {})
```

**Parameters:**

- `cmd` (RemoteCommandType): No description
- `{}` (const CommandData& data =): No description

**Returns:**

bool - 

---

#### registerCommand

```cpp
bool registerCommand(const char* command, JsonCommandCallback callback)
```

**Parameters:**

- `command` (const char*): No description
- `callback` (JsonCommandCallback): No description

**Returns:**

bool - 

---

#### resetConnection

```cpp
void resetConnection()
```

**Returns:**

void - 

---

#### sendAck

```cpp
void sendAck(MessageType cmdType)
```

**Parameters:**

- `cmdType` (MessageType): No description

**Returns:**

void - 

---

#### sendError

```cpp
void sendError(uint8_t errorCode, const String& message = "")
```

**Parameters:**

- `errorCode` (uint8_t): No description
- `""` (const String& message =): No description

**Returns:**

void - 

---

#### sendHeartbeat

```cpp
void sendHeartbeat()
```

**Returns:**

void - 

---

#### sendJsonResponse

```cpp
void sendJsonResponse(const JsonDocument& json)
```

**Parameters:**

- `json` (const JsonDocument&): No description

**Returns:**

void - 

---

#### sendLog

```cpp
void sendLog(const String& message, bool isError = false)
```

**Parameters:**

- `message` (const String&): No description
- `false` (bool isError =): No description

**Returns:**

void - 

---

#### sendPacket

```cpp
bool sendPacket(MessageType type, const void* data = nullptr, uint16_t length = 0)
```

**Parameters:**

- `type` (MessageType): No description
- `nullptr` (const void* data =): No description
- `0` (uint16_t length =): No description

**Returns:**

bool - 

---

#### sendRobotStatus

```cpp
void sendRobotStatus()
```

**Returns:**

void - 

---

#### sendStatus

```cpp
void sendStatus(bool force = false)
```

**Parameters:**

- `false` (bool force =): No description

**Returns:**

void - 

---

#### setConnectionState

```cpp
void setConnectionState(ConnectionState state)
```

**Parameters:**

- `state` (ConnectionState): No description

**Returns:**

void - 

---

#### setRemoteControlEnabled

```cpp
void setRemoteControlEnabled(bool enable)
```

**Parameters:**

- `enable` (bool): No description

**Returns:**

void - 

---

#### unregisterCommand

```cpp
void unregisterCommand(const char* command)
```

**Parameters:**

- `command` (const char*): No description

**Returns:**

void - 

---

#### update

```cpp
void update()
```

**Returns:**

void - 

---

---

[← Back to Index](../README.md) | [↑ MowerArduino Documentation](../README.md)
