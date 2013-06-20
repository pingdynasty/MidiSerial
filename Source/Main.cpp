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

#define IOSSIOSPEED _IOW('T', 2, speed_t)

// #define DEFAULT_SPEED B38400
#define DEFAULT_SPEED 31250
#define DEFAULT_PORT "/dev/ttyS1"
#define BUFFER_LENGTH 2048

class MidiSerial : public juce::MidiInputCallback {
private:
  int m_fd;
  struct termios m_oldtio;
  juce::String m_port;
  int m_speed;
  bool m_verbose;
  bool m_connected, m_running;
  MidiOutput* m_midiout;
  MidiInput* m_midiin;

  juce::String print(const MidiMessage& msg){
    juce::String str;
    for(int i=0; i<msg.getRawDataSize(); ++i){
      str += " 0x";
      str += juce::String::toHexString(msg.getRawData()[i]);
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
    if(m_verbose)
      std::cout << "tx " << m_port << ": " << print(msg) << std::endl;
    if(write(m_fd, msg.getRawData(), msg.getRawDataSize()) != msg.getRawDataSize())
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
    fcntl(m_fd, F_SETFL, 0); // set blocking read
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
    ssize_t len;
    unsigned char buf[BUFFER_LENGTH];
    int used = 0;
    int frompos = 0;
    int topos = 0;
    while(m_running) {
      len = read(m_fd, &buf[topos], BUFFER_LENGTH-topos);
      topos += len;
      len = topos-frompos;
      while(len > 0){ // shortest MIDI message is 1 byte long
        msg = juce::MidiMessage(&buf[frompos], len, used, msg.getRawData()[0]);
        if(m_midiout != NULL)
          m_midiout->sendMessageNow(msg);
        if(m_verbose)
          std::cout << "rx " << m_port << ": " << print(msg) << std::endl;
// 	std::cout << "rx:" << frompos << "-" << topos << " " << used << "/" << len << std::endl;
        if(used == len)
          frompos = topos = 0;
        else
          frompos += used;
	len = topos-frompos;
      }
      if(topos >= BUFFER_LENGTH){
        std::cerr << "buffer overflow!" << std::endl;
        frompos = topos = 0;
      }
    }
    return 0;
  }

  int openSerial(const char* serialport, int baud) {
    struct termios toptions;
    int fd;
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)  {
      perror(serialport);
      return -1;
    }
    if(tcgetattr(fd, &toptions) < 0) {
      perror(serialport);
      return -1;
    }
    m_oldtio = toptions;
    cfsetispeed(&toptions, baud);
    cfsetospeed(&toptions, baud);
    //     cfmakeraw(&tio);
    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw
    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;
    if(ioctl(fd, IOSSIOSPEED, &baud ) == -1){
      perror(serialport);
    }
    if(tcsetattr(fd, TCSANOW, &toptions) < 0) {
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
    m_speed(DEFAULT_SPEED){
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
