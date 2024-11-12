#ifndef YL800N_h
#define YL800N_h

#include <Arduino.h>
#include <assert.h>

//TODO Use CircularBuffer also for Frame to reduce memory footprint

void printByteInHex(const byte b) {
  if (b < 0x10) {
    Serial.print("0");  // Add leading zero for single digit hex values
  }
  Serial.print(b, HEX);
}

void printBytesInHex(const byte* data, int length) {
  for (int i = 0; i < length; ++i) {
    printByteInHex(data[i]);
    Serial.print(" ");
  }
  Serial.println();
}

class CircularBuffer {
public:
  CircularBuffer(size_t capacity)
    : capacity(capacity), head(0), tail(0), count(0) {
    buffer = new byte[capacity];  // Reserva memoria para el búfer
  }

  ~CircularBuffer() {
    delete[] buffer;  // Libera la memoria del búfer
  }

  // Agrega un byte a la cola, devuelve true si se pudo agregar
  bool enqueue(byte data) {
    if (isFull()) {
      return false;  // No se puede agregar porque la cola está llena
    }
    buffer[head] = data;
    head = (head + 1) % capacity;
    count++;
    return true;
  }

  // Elimina un byte de la cola, devuelve true si se pudo eliminar
  bool dequeue(byte& data) {
    if (isEmpty()) {
      return false;  // No se puede extraer porque la cola está vacía
    }
    data = buffer[tail];
    tail = (tail + 1) % capacity;
    count--;
    return true;
  }

  // "Echa un vistazo" al byte en la posición de `tail + offset` sin extraerlo
  bool peek(byte& data, size_t offset = 0) const {
    if (isEmpty() || offset >= count) {
      return false;  // No hay suficientes datos para hacer `peek`
    }
    size_t peekIndex = (tail + offset) % capacity;  // Calcula el índice con el offset
    data = buffer[peekIndex];
    return true;
  }

  // Salta `N` posiciones en la cola sin extraer datos
  bool skip(size_t positions) {
    if (positions > count) {
      return false;  // No se puede saltar más allá de los elementos disponibles
    }
    tail = (tail + positions) % capacity;
    count -= positions;
    return true;
  }

  // Verifica si la cola está vacía
  bool isEmpty() const {
    return count == 0;
  }

  // Verifica si la cola está llena
  bool isFull() const {
    return count == capacity;
  }

  // Devuelve el número de elementos en la cola
  size_t size() const {
    return count;
  }

  // Limpia la cola
  void clear() {
    head = tail = count = 0;
  }

private:
  byte* buffer;     // Puntero al búfer circular
  size_t capacity;  // Capacidad total de la cola
  size_t head;      // Índice de inserción
  size_t tail;      // Índice de extracción
  size_t count;     // Número de elementos actuales en la cola
};

class Frame {
private:
  byte* payload;
  int payload_length;
  int max_size;

public:
  Frame(int max_size)
    : payload(new byte[max_size]), payload_length(0), max_size(max_size) {}
  ~Frame() {
    delete[] payload;
  }
  void sendToStream(Stream& output) {
    output.write(payload, payload_length);
    #ifdef _YL800N_DEBUG_
      Serial.print("> ");
      printBytesInHex(payload, payload_length);
    #endif
  }
  void write(byte b) {
    assert(payload_length < max_size);
    payload[payload_length++] = b;
  }
  void write(byte* buffer, int size) {
    assert(payload_length + size <= max_size);
    memcpy(&payload[payload_length], buffer, size);
    payload_length += size;
  }
  void clear() {
    payload_length = 0;
  }
  byte* getPayload() {
    return payload;
  }
  int getLength() {
    return payload_length;
  }
};

class SERIAL_PARAMETERS {
public:
  enum BAUDRATE {
    BAUDRATE_1200 = 1,
    BAUDRATE_2400 = 2,
    BAUDRATE_4800 = 3,
    BAUDRATE_9600 = 4,
    BAUDRATE_14400 = 5,
    BAUDRATE_19200 = 6,
    BAUDRATE_28800 = 7,
    BAUDRATE_38400 = 8,
    BAUDRATE_57600 = 9,
    BAUDRATE_76800 = 10,
    BAUDRATE_115200 = 11,
    BAUDRATE_230400 = 12
  };

  enum PARITY {
    PARITY_NONE = 0,
    PARITY_EVEN = 1,
    PARITY_ODD = 2
  };

  enum STOP_BITS {
    STOP_BITS_1 = 0,
    STOP_BITS_2 = 1
  };

  SERIAL_PARAMETERS(BAUDRATE baudrate, PARITY parity, STOP_BITS stopbits) {
    this->baudrate = baudrate;
    this->parity = parity;
    this->stopbits = stopbits;
  }

  int value() {
    return (baudrate << 4) | (parity << 1) | stopbits;
  }

  static SERIAL_PARAMETERS decoder(int data) {
    BAUDRATE baudrate = static_cast<BAUDRATE>(data >> 4);
    PARITY parity = static_cast<PARITY>((data >> 1) & 0b11);
    STOP_BITS stopbits = static_cast<STOP_BITS>(data & 0b1);
    return SERIAL_PARAMETERS(baudrate, parity, stopbits);
  }

private:
  BAUDRATE baudrate;
  PARITY parity;
  STOP_BITS stopbits;
};

class FRAME_MODULE_CONFIG {
public:
  enum CHANNEL {
    CH431M = 0,
    CH432M = 1,
    CH429M = 2,
    CH433M = 3,
    CH436M = 4,
    CH434M = 5,
    CH437M = 6,
    CH435M = 7
  };

  enum TX_POWER {
    PWR20dBm = 0,
    PWR17dBm = 1,
    PWR15dBm = 2,
    PWR13dBm = 3,
    PWR11dBm = 4,
    PWR9dBm = 5,
    PWR7dBm = 6,
    PWR5dBm = 7
  };

  enum USER_MODE {
    HEXADECIMAL = 0,
    TRANSPARENT = 1
  };

  enum ROLE {
    SLAVE = 0,
    MASTER = 1
  };

  FRAME_MODULE_CONFIG(CHANNEL channel, USER_MODE user_mode, ROLE role, int network_flag, int node_flag, const SERIAL_PARAMETERS& serial_parameters, TX_POWER tx_power = PWR20dBm, int bandwidth = 9, int spread_factor = 9)
    : channel(channel), user_mode(user_mode), role(role), network_flag(network_flag), node_flag(node_flag), serial_parameters(serial_parameters), tx_power(tx_power), bandwidth(bandwidth), spread_factor(spread_factor) {}

  encode(Frame& frame) {
    encoder(frame, channel, user_mode, role, network_flag, node_flag, serial_parameters, tx_power, bandwidth, spread_factor);
  }

  static encoder(Frame& frame, CHANNEL channel, USER_MODE user_mode, ROLE role, int network_flag, int node_flag, const SERIAL_PARAMETERS& serial_parameters, TX_POWER tx_power = PWR20dBm, int bandwidth = 9, int spread_factor = 9) {
    frame.write((byte)0xA5);
    frame.write((byte)0xA5);
    frame.write(channel);
    frame.write(tx_power);
    frame.write(user_mode);
    frame.write(role);
    frame.write(network_flag & 0xFF);
    frame.write((network_flag >> 8) & 0xFF);
    frame.write(node_flag & 0xFF);
    frame.write((node_flag >> 8) & 0xFF);
    frame.write((byte)0x00);
    frame.write((byte)0x00);
    frame.write((byte)0x02);
    frame.write(serial_parameters.value());
    frame.write(bandwidth);
    frame.write(spread_factor);
  }

  static FRAME_MODULE_CONFIG decoder(const byte* data) {
    CHANNEL channel = static_cast<CHANNEL>(data[3]);
    TX_POWER tx_power = static_cast<TX_POWER>(data[4]);
    USER_MODE user_mode = static_cast<USER_MODE>(data[5]);
    ROLE role = static_cast<ROLE>(data[6]);
    int network_flag = (data[7] << 8) | data[8];
    int node_flag = (data[9] << 8) | data[10];
    SERIAL_PARAMETERS serial_parameters = SERIAL_PARAMETERS::decoder(data[11]);
    int bandwidth = data[12];
    int spread_factor = data[13];
    return FRAME_MODULE_CONFIG(channel, user_mode, role, network_flag, node_flag, serial_parameters, tx_power, bandwidth, spread_factor);
  }

private:
  CHANNEL channel;
  TX_POWER tx_power;
  USER_MODE user_mode;
  ROLE role;
  int network_flag;
  int node_flag;
  SERIAL_PARAMETERS serial_parameters;
  int bandwidth;
  int spread_factor;
};

class FRAME_APPLICATION_DATA {
public:
  enum class WAIT_ACK : byte {
    DISABLED = 0x00,
    ENABLED = 0x01
  };

  enum class ROUTE_DISCOVERY : byte {
    DISABLED = 0x00,
    AUTOMATIC = 0x01,
    FORCED = 0x02
  };

  FRAME_APPLICATION_DATA(int target_address, WAIT_ACK wait_ack, int max_hops, ROUTE_DISCOVERY route_discovery, const byte* payload, int payload_length)
    : target_address(target_address), wait_ack(wait_ack), max_hops(max_hops), route_discovery(route_discovery), payload_length(payload_length) {
    this->payload = new byte[payload_length];
    memcpy(this->payload, payload, payload_length);
  }

  ~FRAME_APPLICATION_DATA() {
    delete[] payload;
  }

  int getLength() {
    return 6 + payload_length;
  }

  encode(Frame& frame) {
    encoder(frame, target_address, wait_ack, max_hops, route_discovery, payload, payload_length);
  }

  static encoder(Frame& frame, int target_address, WAIT_ACK wait_ack, int max_hops, ROUTE_DISCOVERY route_discovery, const byte* payload, int payload_length) {
    frame.write(target_address & 0xFF);
    frame.write((target_address >> 8) & 0xFF);
    frame.write(static_cast<byte>(wait_ack));
    frame.write(max_hops);
    frame.write(static_cast<byte>(route_discovery));
    frame.write(payload_length);
    frame.write(payload, payload_length);
  }

private:
  int target_address;
  WAIT_ACK wait_ack;
  int max_hops;
  ROUTE_DISCOVERY route_discovery;
  int payload_length;
  byte* payload;
};

class FRAME_APPLICATION_DATA_RECEIVE {
public:
  FRAME_APPLICATION_DATA_RECEIVE(int target_address, int signal_strength, int payload_length, const byte* payload)
    : target_address(target_address), signal_strength(signal_strength), payload_length(payload_length) {
    this->payload = new byte[payload_length];
    memcpy(this->payload, payload, payload_length);
  }

  ~FRAME_APPLICATION_DATA_RECEIVE() {
    delete[] payload;
  }

  static FRAME_APPLICATION_DATA_RECEIVE decoder(const byte* data) {
    int target_address = (data[0] << 8) | data[1];
    int signal_strength = data[2];
    int payload_length = data[3];
    const byte* payload = &data[4];
    return FRAME_APPLICATION_DATA_RECEIVE(target_address, signal_strength, payload_length, payload);
  }

private:
  int target_address;
  int signal_strength;
  int payload_length;
  byte* payload;
};

class FRAME {
public:
  static byte SEQUENCE_NUMBER;

  enum class FRAME_TYPE : byte {
    MODULE_CONFIG = 0x01,
    MAC_TESTING = 0x02,
    NET_TESTING = 0x03,
    DEBUG = 0x04,
    APPLICATION_DATA = 0x05
  };

  class COMMAND_TYPE {
  public:
    enum class MODULE_CONFIG : byte {
      WRITE_CONFIG = 0x01,
      READ_CONFIG = 0x02,
      VERSION = 0x06,
      RESET = 0x07,
      WRITE_CONFIG_RESPONSE = 0x81,
      READ_CONFIG_RESPONSE = 0x82,
      VERSION_RESPONSE = 0x86,
      RESET_RESPONSE = 0x87
    };

    enum class DEBUG : byte {
      WRITE_ACCESS_CONTROL_LIST = 0x01,
      READ_ACCESS_CONTROL_LIST = 0x02,
      WRITE_ACCESS_CONTROL_LIST_RESPONSE = 0x81,
      READ_ACCESS_CONTROL_LIST_RESPONSE = 0x82
    };

    enum class APPLICATION_DATA : byte {
      SEND = 0x01,
      ROUTE_DISCOVERY = 0x08,
      SEND_RESPONSE = 0x81,
      RECEIVE = 0x82,
      ROUTE_DISCOVERY_RESPONSE = 0x88
    };
  };

  static byte crc(const byte* data, int length) {
    byte crc = 0;
    for (int i = 0; i < length; ++i) {
      crc ^= data[i];
    }
    return crc;
  }

  static encoder(Frame& frame, FRAME_TYPE frame_type, byte command_type, void* payload, int payload_length) {
    frame.write(static_cast<byte>(frame_type));
    frame.write(SEQUENCE_NUMBER);
    frame.write(command_type);
    frame.write(payload_length);
    frame.write(payload, payload_length);
    frame.write(crc(frame.getPayload(), 4 + payload_length));
  }

  static encoder(Frame& frame, FRAME_MODULE_CONFIG& config) {
    frame.write(static_cast<byte>(FRAME_TYPE::MODULE_CONFIG));
    frame.write(SEQUENCE_NUMBER);
    frame.write(static_cast<byte>(COMMAND_TYPE::MODULE_CONFIG::WRITE_CONFIG));
    frame.write(16);  // payload_length
    config.encode(frame);
    frame.write(crc(frame.getPayload(), frame.getLength()));  //TODO Check CRC
  }

  static encoder(Frame& frame, FRAME_APPLICATION_DATA& data) {
    frame.write(static_cast<byte>(FRAME_TYPE::APPLICATION_DATA));
    frame.write(SEQUENCE_NUMBER);
    frame.write(static_cast<byte>(COMMAND_TYPE::APPLICATION_DATA::SEND));
    frame.write(data.getLength());  // payload_length
    data.encode(frame);
    frame.write(crc(frame.getPayload(), frame.getLength()));  //TODO Check CRC
  }

  //TODO Implementar una funcion que no necesite crear un objeto de tipo FRAME_APPLICATION_DATA para enviar.
};

byte FRAME::SEQUENCE_NUMBER = 0x00;

class YL800N {
public:
  YL800N(Stream& serial = Serial)
    : serial(serial), output(100), input(100) {}

  void readInput() {
    while (serial.available()) {
      char c = serial.read();
      input.enqueue(c);
    }
  }

  void skip(int n) {
    #ifdef _YL800N_DEBUG_
      Serial.print("< ");
    #endif 

    for (int i = 0; i < n; i++) {
      byte b;
      input.dequeue(b);
      #ifdef _YL800N_DEBUG_
        printByteInHex(b);
        Serial.print(" ");
      #endif
    }

    #ifdef _YL800N_DEBUG_
      Serial.println();
    #endif
  }

  // 0 NO FRAMES
  // 1 DATA AVAILABLE
  // 2 FRAME SKIPPED, PROBABLY MORE FRAMES
  int processFrames(String* str = nullptr) {
    if (input.size() < 5) {
      //Not enough data for a complete frame
      return 0;
    }

    byte frameType;
    byte commandType;
    byte payloadLength;

    input.peek(frameType, 0);
    input.peek(commandType, 2);
    input.peek(payloadLength, 3);

    switch (frameType) {
      case static_cast<byte>(FRAME::FRAME_TYPE::APPLICATION_DATA):
        switch (commandType) {
          case static_cast<byte>(FRAME::COMMAND_TYPE::APPLICATION_DATA::SEND_RESPONSE):
            #ifdef _YL800N_DEBUG_
              Serial.print("SEND_RESPOSE ");
            #endif
            skip(5 + payloadLength);
            break;
          case static_cast<byte>(FRAME::COMMAND_TYPE::APPLICATION_DATA::RECEIVE):
            if (str != nullptr) {       
              #ifdef _YL800N_DEBUG_                  
                Serial.print("RECEIVE < ");
              #endif                 
              str->reserve(payloadLength);
              input.skip(4); // 4 bytes header
              byte addr1, addr2;
              input.dequeue(addr1);
              input.dequeue(addr2);
              input.skip(2); // 2 bytes = signal strength + payload_length
              for (int i = 0; i < payloadLength - 4; i++) {  // 4 bytes data (address+signal+payload_length)
                byte b;
                input.dequeue(b);
                #ifdef _YL800N_DEBUG_
                  printByteInHex(b);
                  Serial.print(" ");
                #endif
                str->concat((char)b);
              }
              input.skip(1);
              #ifdef _YL800N_DEBUG_
                Serial.print("[");
                Serial.print(*str);
                Serial.print("] from ");
                int address = (addr2 << 8) | addr1;
                Serial.println(address);
              #endif
              return 1;
            } else {
              #ifdef _YL800N_DEBUG_
                Serial.println("** DATA AVAILABLE **");
              #endif
              return 1;
            }
            break;
          default:
            skip(5 + payloadLength);
            break;
        }
        break;
      default:
        skip(5 + payloadLength);
        break;
    }
    return 2;
  }

  bool processInput(String* str = nullptr) {
    int result;
    do {
      result = processFrames(str);
    } while(result==2);
    return result == 1;
  }

  void skipAndDebug() {
    readInput();
    processInput();
  }

  bool readLine(String& line) {
    line.remove(0);
    readInput();
    return processInput(&line);
  }

  void setConfig(
    FRAME_MODULE_CONFIG::CHANNEL channel,
    FRAME_MODULE_CONFIG::USER_MODE user_mode,
    FRAME_MODULE_CONFIG::ROLE role,
    int network_flag,
    int node_flag,
    FRAME_MODULE_CONFIG::TX_POWER tx_power = FRAME_MODULE_CONFIG::TX_POWER::PWR20dBm,
    int bandwidth = 9,
    int spread_factor = 9) {
    SERIAL_PARAMETERS serial_parameters(
      SERIAL_PARAMETERS::BAUDRATE::BAUDRATE_9600,
      SERIAL_PARAMETERS::PARITY::PARITY_NONE,
      SERIAL_PARAMETERS::STOP_BITS::STOP_BITS_1);

    FRAME_MODULE_CONFIG module_config(
      channel,
      user_mode,
      role,
      network_flag,
      node_flag,
      serial_parameters,
      tx_power,
      bandwidth,
      spread_factor);

    output.clear();
    FRAME::encoder(output, module_config);
    output.sendToStream(serial);

    skipAndDebug();
  }

  void sendString(
    const String& payload,
    int target_address = 0xFFFF,
    FRAME_APPLICATION_DATA::WAIT_ACK wait_ack = FRAME_APPLICATION_DATA::WAIT_ACK::DISABLED,
    int max_hops = 7,
    FRAME_APPLICATION_DATA::ROUTE_DISCOVERY route_discovery = FRAME_APPLICATION_DATA::ROUTE_DISCOVERY::AUTOMATIC) {
    sendData(
      target_address,
      reinterpret_cast<const byte*>(payload.c_str()),
      payload.length(),
      wait_ack,
      max_hops,
      route_discovery);
  }

  void sendData(
    int target_address,
    const byte* payload,
    int payload_length,
    FRAME_APPLICATION_DATA::WAIT_ACK wait_ack = FRAME_APPLICATION_DATA::WAIT_ACK::DISABLED,
    int max_hops = 7,
    FRAME_APPLICATION_DATA::ROUTE_DISCOVERY route_discovery = FRAME_APPLICATION_DATA::ROUTE_DISCOVERY::AUTOMATIC) {

    FRAME_APPLICATION_DATA app_data(
      target_address,
      wait_ack,
      max_hops,
      route_discovery,
      payload,
      payload_length);

    output.clear();
    FRAME::encoder(output, app_data);
    output.sendToStream(serial);

    skipAndDebug();
  }

  void print(const String& s) { sendString(s); }
  void println(const String& s) { sendString(s); }

private:
  Stream& serial;
  Frame output;
  CircularBuffer input;  
};

#endif

