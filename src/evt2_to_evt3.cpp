// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

static void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "evt2_to_evt3 -i name_of_evt2_raw_file -o name_of_evt3_raw_file" << std::endl;
}

class EVT3Encoder
{
  enum Code {
    ADDR_Y = 0,
    ADDR_X = 2,
    VECT_BASE_X = 3,
    VECT_8 = 5,
    TIME_LOW = 6,
    TIME_HIGH = 8,
    EXT_TRIG = 10
  };

  struct __attribute__((packed)) TimeLow
  {
    TimeLow(uint16_t ts_usec) : t(ts_usec), code(Code::TIME_LOW) {}
    uint16_t t : 12;
    uint16_t code : 4;
  };

  struct __attribute__((packed)) TimeHigh
  {
    TimeHigh(uint16_t ts) : t(ts), code(Code::TIME_HIGH) {}
    uint16_t t : 12;
    uint16_t code : 4;
  };

  struct __attribute__((packed)) AddrY
  {
    AddrY(uint16_t ya, uint16_t st) : y(ya), system_type(st), code(Code::ADDR_Y) {}
    uint16_t y : 11;
    uint16_t system_type : 1;
    uint16_t code : 4;
  };

  struct __attribute__((packed)) AddrX
  {
    AddrX(uint16_t xa, uint8_t p) : x(xa), polarity(p), code(Code::ADDR_X) {}
    uint16_t x : 11;
    uint16_t polarity : 1;
    uint16_t code : 4;
  };

  struct __attribute__((packed)) VectBaseX
  {
    VectBaseX(uint16_t xa, uint16_t pa) : x(xa), polarity(pa), code(Code::VECT_BASE_X) {}
    uint16_t x : 11;
    uint16_t polarity : 1;
    uint16_t code : 4;
  };

  struct __attribute__((packed)) Vect8
  {
    Vect8(uint16_t v) : valid(v), code(Code::VECT_8) {}
    uint16_t valid : 8;
    uint16_t unused : 4;
    uint16_t code : 4;
  };

  struct __attribute__((packed)) ExtTrigger
  {
    ExtTrigger(uint16_t v, uint16_t c) : value(v), channel(c), code(Code::EXT_TRIG) {}
    uint16_t value : 1;
    uint16_t unused : 7;
    uint16_t channel : 4;
    uint16_t code : 4;
  };

public:
  EVT3Encoder(const std::string & out_name)
  {
    currentMask_.reset();
    out_stream_.open(out_name, std::ios::out | std::ios::binary);
  };
  ~EVT3Encoder()
  {
    std::cout << "min y: " << min_y_ << std::endl;
    std::cout << "max y: " << max_y_ << std::endl;
    std::cout << "min x: " << min_x_ << std::endl;
    std::cout << "max x: " << max_x_ << std::endl;
  }
  void setMasterSlave(uint16_t ms) { master_slave_ = ms; }

  void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity)
  {
#if 0
    std::cout << "event: " << sensor_time << " x: " << ex << " y: " << ey
              << " p: " << (int)(polarity) << std::endl;
#endif
    if (ey > max_y_) {
      max_y_ = ey;
    }
    if (ey < min_y_) {
      min_y_ = ey;
    }
    if (ex > max_x_) {
      max_x_ = ex;
    }
    if (ex < min_x_) {
      min_x_ = ex;
    }
    handleTime(sensor_time);
    if (ey != currentY_) {
      flushState();
      currentY_ = ey;
      addToEvents(AddrY(currentY_, master_slave_));
    }
    if (polarity != currentPolarity_) {
      flushState();
      currentPolarity_ = polarity;
    }

    const uint16_t x_high = ex & X_HIGH_MASK;
    const uint16_t x_low = ex & X_LOW_MASK;
    if (x_high != currentXHigh_) {
      flushState();
      currentXHigh_ = x_high;
      currentX_ = ex;
    }
    // mark the current bit in the mask
    currentMask_.set(x_low, 1);
  }
  void eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id)
  {
    handleTime(sensor_time);
    addToEvents(ExtTrigger(edge, id));
  }

  void writeRaw(const std::string & s)
  {
    const auto repl = std::regex_replace(s, std::regex("evt 2.0"), "evt 3.0");
    out_stream_.write(reinterpret_cast<const char *>(repl.data()), repl.size());
  }

  void flush()
  {
    flushState();
    out_stream_.write(reinterpret_cast<const char *>(event_buffer_.data()), event_buffer_.size());
    event_buffer_.clear();
  }

private:
  void handleTime(uint64_t sensor_time)
  {
    if (!has_valid_high_time_) {
      has_valid_high_time_ = true;
      timeHigh_ = sensor_time & TIME_HIGH_MASK;
      addToEvents(TimeHigh(static_cast<uint16_t>(timeHigh_ >> 12)));
    }
    if ((sensor_time & TIME_HIGH_MASK) != timeHigh_) {
      timeHigh_ = sensor_time & TIME_HIGH_MASK;
      flushState();
      addToEvents(TimeHigh(static_cast<uint16_t>(timeHigh_ >> 12)));
    }
    if ((sensor_time & TIME_LOW_MASK) != timeLow_) {
      timeLow_ = sensor_time & TIME_LOW_MASK;
      flushState();
      addToEvents(TimeLow(static_cast<uint16_t>(timeLow_)));
    }
  }
  void flushState()
  {
    if (currentMask_.any()) {
      if (currentMask_.count() == 1) {
        addToEvents<AddrX>(AddrX(currentX_, currentPolarity_));
      } else {
        addToEvents(VectBaseX(currentXHigh_, currentPolarity_));
        addToEvents(Vect8(static_cast<uint16_t>(currentMask_.to_ulong())));
      }
      currentMask_.reset();
    }
  }
  template <class T>
  void addToEvents(const T & token)
  {
    const uint8_t * p = reinterpret_cast<const uint8_t *>(&token);
    event_buffer_.push_back(p[0]);  // token must be 16 bytes or else!
    event_buffer_.push_back(p[1]);
  }
  std::fstream out_stream_;
  // ---- state for the encoder
  uint64_t timeHigh_{0};  // is in shifted state, i.e. only bits 23-12 are non-zero
  uint64_t timeLow_{0};
  uint16_t currentY_ = std::numeric_limits<uint16_t>::max();
  uint16_t currentX_ = std::numeric_limits<uint16_t>::max();
  uint16_t currentXHigh_{0};
  uint8_t currentPolarity_{2};  // initialize to out-of-range value
  std::bitset<8> currentMask_;
  std::vector<uint8_t> event_buffer_;  // encoder buffered output
  bool has_valid_high_time_{false};
  uint16_t master_slave_{0};  // default to master
  // --- debugging stuff
  uint16_t max_y_{0};
  uint16_t min_y_ = std::numeric_limits<uint16_t>::max();
  uint16_t max_x_{0};
  uint16_t min_x_ = std::numeric_limits<uint16_t>::max();
  // --- masks
  static constexpr uint64_t TIME_MASK = ((1ULL << 24) - 1ULL);
  static constexpr uint64_t TIME_LOW_MASK = ((1ULL << 12) - 1ULL);
  static constexpr uint64_t TIME_HIGH_MASK = TIME_MASK & (~TIME_LOW_MASK);
  static constexpr uint16_t X_LOW_MASK = 0x0007;        // lowest 3 bits
  static constexpr uint16_t X_HIGH_MASK = ~X_LOW_MASK;  // highest 13 bits
};

class EVT2Decoder
{
public:
  enum EVTType {
    CD_OFF = 0,
    CD_ON = 1,
    EVT_TIME_HIGH = 8,
    EXT_TRIGGER = 10,
    OTHERS = 0xe,
    CONTINUED = 0xf
  };
  EVT2Decoder() = default;
  void setHandler(EVT3Encoder * h) { handler_ = h; }

  void process(const char * data, size_t n)
  {
    for (const char * p = data; p < data + n; p += 4) {
      const uint32_t & e = *reinterpret_cast<const uint32_t *>(p);
      const uint32_t typ = (e & 0xf0000000) >> 28;
      switch (typ) {
        case CD_OFF:
          num_events_[0]++;
          if (has_valid_high_time_) {
            handler_->eventCD(makeTime(e), makeX(e), makeY(e), 0);
          }
          break;
        case CD_ON:
          num_events_[1]++;
          if (has_valid_high_time_) {
            handler_->eventCD(makeTime(e), makeX(e), makeY(e), 1);
          }
          break;
        case EVT_TIME_HIGH:
          time_high_ = (static_cast<uint64_t>(e) & ((1ULL << 28) - 1LL)) << 6;
          if (!has_valid_high_time_) {
            has_valid_high_time_ = true;
          }
          break;
        case EXT_TRIGGER:
          num_triggers_++;
          if (has_valid_high_time_) {
            handler_->eventExtTrigger(makeTime(e), (e & 0x1), (e & (0x1f << 8)) >> 8);
          }
          break;
        case OTHERS:
          //  std::cout << "others event!" << std::endl;
          break;
        case CONTINUED:
          // std::cout << "continued event!" << std::endl;
          break;
      }
    }
  }
  const std::array<size_t, 2> & getNumEvents() const { return (num_events_); }
  size_t getNumTriggers() const { return (num_triggers_); }

private:
  uint64_t makeTime(uint32_t e) const { return (time_high_ | ((e & TS_MASK) >> 22)); }
  static uint16_t makeX(uint32_t e) { return ((e & (0x7ff << 11)) >> 11); }
  static uint16_t makeY(uint32_t e) { return (e & 0x7ff); }
  EVT3Encoder * handler_{nullptr};
  std::array<size_t, 2> num_events_{{0, 0}};
  size_t num_triggers_{0};
  uint64_t time_high_{0};
  bool has_valid_high_time_{false};
  static const uint32_t TS_MASK = 0x3f << 22;
};

class Processor
{
public:
  Processor(const std::string & out_name) : encoder_(out_name) { decoder_.setHandler(&encoder_); }
  void process(const char * data, size_t n) { decoder_.process(data, n); }
  void flush() { encoder_.flush(); }
  const std::array<size_t, 2> & getNumEvents() const { return (decoder_.getNumEvents()); }
  size_t getNumTriggers() const { return (decoder_.getNumTriggers()); }
  void copyHeader(std::iostream & in)
  {
    int c;
    while ((c = in.peek()) == '%') {
      std::string line;
      std::getline(in, line);
      encoder_.writeRaw(line);
      encoder_.writeRaw("\n");
    }
  }

private:
  EVT2Decoder decoder_;
  EVT3Encoder encoder_;
};

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  int bufSize(150000);
  while ((opt = getopt(argc, argv, "B:i:o:")) != -1) {
    switch (opt) {
      case 'o':
        outFile = optarg;
        break;
      case 'i':
        inFile = optarg;
        break;
      case 'B':
        bufSize = 4 * atoi(optarg);  // each EVT2 event has 4 bytes!
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (inFile.empty() || outFile.empty()) {
    std::cout << "missing input or output file name!" << std::endl;
    usage();
    return (-1);
  }
  std::fstream in;
  in.open(inFile, std::ios::in | std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "cannot open file: " << inFile << std::endl;
    return (-1);
  }
  std::vector<char> buffer(bufSize);
  size_t bytesRead(0);
  Processor processor(outFile);
  processor.copyHeader(in);
  const auto start = std::chrono::high_resolution_clock::now();
  do {
    in.read(&(buffer[0]), buffer.size());
    if (in.gcount() != 0) {
      processor.process(reinterpret_cast<char *>(&buffer[0]), in.gcount());
    }
    bytesRead += in.gcount();
    processor.flush();
  } while (in.gcount() >= std::streamsize(buffer.size()));

  const decltype(start) final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  const auto off_events = processor.getNumEvents()[0];
  const auto on_events = processor.getNumEvents()[1];
  const size_t numEvents = on_events + off_events;
  std::cout << "number of bytes read: " << bytesRead / (1024 * 1024) << " MB in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "total number of events: " << numEvents << std::endl;
  std::cout << "on events:  " << on_events << std::endl;
  std::cout << "off events: " << off_events << std::endl;
  std::cout << "trigger events: " << processor.getNumTriggers() << std::endl;
  std::cout << "byte rate: " << static_cast<double>(bytesRead) / total_duration.count() << " MB/s"
            << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mev/s"
            << std::endl;
  return (0);
}