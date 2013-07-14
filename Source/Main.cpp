#include "../JuceLibraryCode/JuceHeader.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <iostream>
#include <sys/ioctl.h>
#include <errno.h>
#include "MidiReader.h"

#define DEFAULT_SPEED              115200
#define DEFAULT_PORT               "/dev/ttyS1"
#define DEFAULT_BUFFER_SIZE        4096

// class MidiReader {
// private:
// public:
//   int readMidiMessage(juce::MidiMessage& msg, unsigned char lastbyte, unsigned char* buf, ssize_t len){
//     int used;
//     if(buf[0] == 0xf7){
//       // sysex
//     }else{
//       msg = juce::MidiMessage(buf, len, used, lastbyte);
//     }
//     return used;
//   }
//   int push(unsigned char data){
//   }
// };

class MidiSerial : public juce::MidiInputCallback {
private:
  int m_fd;
  int bufferSize;
  struct termios m_oldtio;
  juce::String m_port;
  int m_speed;
  bool m_verbose;
  bool m_connected, m_running;
  MidiOutput* m_midiout;
  MidiInput* m_midiin;
  MidiReader midireader;

  juce::String print(const MidiMessage& msg){
    juce::String str;
    for(int i=0; i<msg.getRawDataSize(); ++i){
      str += " 0x";
      str += juce::String::toHexString(msg.getRawData()[i]);
    }
    return str;
  }

  juce::String print(const unsigned char* buf, int len){
    juce::String str;
    for(int i=0; i<len; ++i){
      str += " 0x";
      str += juce::String::toHexString(buf[i]);
    }
    return str;
  }

  void listDevices(const StringArray& names){
    for(int i=0; i<names.size(); ++i)
      std::cout << i << ": " << names[i] << std::endl;
  }

public:
  void handleIncomingMidiMessage(MidiInput *source,
                                 const MidiMessage &msg){
//     if(msg.isSysEx()){
//       handlePartialSysexMessage(source, msg.getRawData(), msg.getRawDataSize(), msg.getTimeStamp());
//     }else{
      if(m_verbose)
	std::cout << "tx " << m_port << ": " << print(msg) << std::endl;
      if(write(m_fd, msg.getRawData(), msg.getRawDataSize()) != msg.getRawDataSize())
	perror(m_port.toUTF8());    
//     }
  }

  void handlePartialSysexMessage(MidiInput* source, const uint8* data,
				 int size, double timestamp){
    if(m_verbose)
      std::cout << "tx " << m_port << ": " << " sysex " << size << " bytes" << std::endl;
    ssize_t len = write(m_fd, data, size);
    if(len != size)
      perror(m_port.toUTF8());    
  }

  void usage(){
    std::cerr << "MidiSerial v1"  << std::endl << "usage:" << std::endl
              << "-p FILE\t set serial port" << std::endl
              << "-s NUM\t set serial speed (default: " << DEFAULT_SPEED << ")" << std::endl
              << "-v\t verbose, prints messages sent/received" << std::endl
              << "-i NUM\t set MIDI input device" << std::endl
              << "-o NUM\t set MIDI output device" << std::endl
              << "-c NAME\t create MIDI input/output device" << std::endl
              << "-l\t list MIDI input/output devices and exit" << std::endl
              << "-h or --help\tprint this usage information and exit" << std::endl;
  }

  int connect(){
    if(m_midiin == NULL && m_midiout == NULL){
      // default behaviour if no interface specified
      m_midiout = MidiOutput::createNewDevice("MidiSerial");
      m_midiin = MidiInput::createNewDevice("MidiSerial", this);
    }
    m_fd = openSerial(m_port.toUTF8(), m_speed);
    if(m_fd < 0){
      perror(m_port.toUTF8()); 
      return -1; 
    }
    if(m_verbose)
      std::cout << "tty " << m_port << " at " << m_speed << " baud" << std::endl;
    //     fcntl(m_fd, F_SETFL, FNDELAY); // set non-blocking read
//     fcntl(m_fd, F_SETFL, 0); // set blocking read
    //     fcntl(fd, F_SETFL, O_APPEND); // append output
    //     fcntl(fd, F_NOCACHE, 1); // turn caching off
    m_connected = true;
    return 0;
  }

  int stop(){
    if(m_verbose)
      std::cout << "stopping" << std::endl;
    m_running = false;
    if(m_midiin != NULL)
      m_midiin->stop();
    return 0;
  }

  int start(){
    if(m_midiin != NULL)
      m_midiin->start();
    m_running = true;
    return 0;
  }

  int run(){
    juce::MidiMessage msg;
    unsigned char buf[bufferSize];
    ssize_t len;
    int frompos;
    MidiReaderStatus status;
    while(m_running) {
      frompos = 0;
      len = read(m_fd, buf, bufferSize);
	/* possibility that buffer contains:
	   a) one incomplete message
	   b) one complete message
	   c) one complete message followed by one or more complete messages, 
	      and/or possibly followed by an incomplete message
	*/
//       use: getc_unlocked() or getc() instead, or getw()?
      while(frompos < len){
	status = midireader.read(buf[frompos++]);
	if(status == READY){
	  msg = midireader.getMessage();
	  if(m_midiout != NULL)
	    m_midiout->sendMessageNow(msg);
	  if(m_verbose)
	    std::cout << "rx " << m_port << ": " << print(msg) << std::endl;
	}
      }
    }
    return 0;
  }

  int openSerial(const char* serialport, int baud) {
    struct termios tio;
    int fd;
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1){
      perror(serialport);
      return -1;
    }
    if(tcgetattr(fd, &tio) < 0){
      perror(serialport);
      return -1;
    }
    m_oldtio = tio;
    fcntl(fd, F_SETFL, 0);

    /* Configure port */
    tio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    //     cfmakeraw(&tio);
	
    tcflush(fd, TCIFLUSH);
    if(tcsetattr(fd, TCSANOW, &tio) < 0){
      perror(serialport);
      return -1;
    }

    return fd;
  }

  int configure(int argc, char* argv[]) {
    for(int i=1; i<argc; ++i){
      juce::String arg = juce::String(argv[i]);
      if(arg.compare("-p") == 0 && ++i < argc){
        m_port = juce::String(argv[i]);
      }else if(arg.compare("-v") == 0){
        m_verbose = true;
      }else if(arg.compare("-l") == 0){
        std::cout << "MIDI output devices:" << std::endl;
        listDevices(MidiOutput::getDevices());
        std::cout << "MIDI input devices:" << std::endl;
        listDevices(MidiInput::getDevices());
        return 1;
      }else if(arg.compare("-s") == 0 && ++i < argc){
        m_speed = juce::String(argv[i]).getIntValue();
      }else if(arg.compare("-o") == 0 && ++i < argc && m_midiout == NULL){
        int index = juce::String(argv[i]).getIntValue();
        m_midiout = MidiOutput::openDevice(index);
        if(m_verbose)
          std::cout << "Opening MIDI output: " << MidiOutput::getDevices()[index] << std::endl;
      }else if(arg.compare("-i") == 0 && ++i < argc && m_midiin == NULL){
        int index = juce::String(argv[i]).getIntValue();
        m_midiin = MidiInput::openDevice(index, this);
        if(m_verbose)
          std::cout << "Opening MIDI input: " << MidiInput::getDevices()[index] << std::endl;
      }else if(arg.compare("-c") == 0 && ++i < argc && m_midiin == NULL && m_midiout == NULL){
        String name = juce::String(argv[i]);
        if(m_verbose)
          std::cout << "Creating MIDI input and output: " << name << std::endl;
        m_midiout = MidiOutput::createNewDevice(name);
        m_midiin = MidiInput::createNewDevice(name, this);
      }else if(arg.compare("-h") == 0 || arg.compare("--help") == 0 ){
        usage();
        return 1;
      }else{
        usage();
        errno = EINVAL;
        perror(arg.toUTF8());
        return -1;
      }
    }
    return 0;
  }

  int disconnect(){
    if(m_connected){
      if(m_verbose)
	std::cout << "disconnecting" << std::endl;
      tcsetattr(m_fd, TCSANOW, &m_oldtio);
      close(m_fd);
      m_connected = false;
    }
    if(m_midiout != NULL)
      delete m_midiout;
    if(m_midiin != NULL)
      delete m_midiin;
    return 0;
  }

  MidiSerial() :
    m_port(DEFAULT_PORT),
    m_speed(DEFAULT_SPEED), 
    bufferSize(DEFAULT_BUFFER_SIZE),
    midireader(DEFAULT_BUFFER_SIZE) {
    m_midiin = NULL;
    m_midiout = NULL;
  }
  
  ~MidiSerial(){
    if(m_running)
      stop();
    if(m_connected)
      disconnect();
  }
};

MidiSerial service;

void sigfun(int sig){
  service.stop();
  (void)signal(SIGINT, SIG_DFL);
}

int main(int argc, char* argv[]) {
//   const ScopedJuceInitialiser_NonGUI juceSystemInitialiser;
  (void)signal(SIGINT, sigfun);
  int ret = service.configure(argc, argv);
  if(!ret)
    ret = service.connect();
  if(!ret)
    ret = service.start();
  if(!ret)
    ret = service.run();
  ret |= service.disconnect();
  return ret;
}
