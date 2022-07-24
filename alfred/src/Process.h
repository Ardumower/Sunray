/*
  Linux processes
*/

#ifndef PROCESS_H_
#define PROCESS_H_

#include "Arduino.h"
#include <Bridge.h>
#include <StringArray.h>

typedef union {
  int pipes[2];
  struct {
    int read;
    int write;
  };
} pipe_t;

typedef struct {
  pipe_t in;
  pipe_t out;
  pipe_t err;
} process_pipes_t;

class Process : public Stream {
  private:
    boolean         success;
    boolean         executed;
    StringArray     cmd_array;
    int             exec_pid;
    int             exec_result;
    process_pipes_t pipes;
    pthread_t       thread;
    boolean         thread_running;

  public:
    // Constructor with a user provided BridgeClass instance
    Process(BridgeClass &_b = Bridge);
    ~Process();
    operator bool () { return success; }

    void begin(const String &command);
    void addParameter(const String &param);
    void close();

    void runAsynchronously();
    boolean running();
    unsigned int exitValue(){ return exec_result; }

    // Stream methods
    // (read from process stdout)
    int available();
    int read();
    int read(char * buf, size_t len);
    int peek(){ return -1; }
    // (write to process stdin)
    size_t write(uint8_t);
    size_t write(uint8_t *data, size_t len);
    void flush(){}
    // (read from process stderr)
    int errAvailable();
    int errRead();
    int errRead(char * buf, size_t len);

    unsigned int run();
    unsigned int runShellCommand(const String &command);
    void runShellCommandAsynchronously(const String &command);

    //called by the created thread
    //will block if called directly
    //and will execute on the same thread
    void execute();
};

#endif
