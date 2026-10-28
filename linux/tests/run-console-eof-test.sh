#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
python3 - "$work/test.cpp" <<'PY'
from pathlib import Path
import sys
s=Path('linux/src/Console.cpp').read_text()
methods=[]
for start,end in [('int LinuxConsole::available()', 'int LinuxConsole::peek()'),('int LinuxConsole::read()', 'void LinuxConsole::flush()')]:
 methods.append(s[s.index(start):s.index(end)])
Path(sys.argv[1]).write_text('''#include <unistd.h>
#include <sys/select.h>
#include <fcntl.h>
#include <cassert>
#include <cstdio>
class LinuxConsole { public: int keyboard; int available(); int read(); };
'''+'\n'.join(methods)+'''
int main(){
 LinuxConsole c;int fd=open("/dev/null",O_RDONLY);c.keyboard=fd;
 assert(c.available());assert(c.read()==-1);assert(!c.available());assert(c.read()==-1);close(fd);
 int pipefd[2];assert(pipe(pipefd)==0);c.keyboard=pipefd[0];
 assert(!c.available());unsigned char value=0xff;assert(write(pipefd[1],&value,1)==1);
 assert(c.available());assert(c.read()==255);assert(!c.available());
 close(pipefd[1]);assert(c.available());assert(c.read()==-1);assert(!c.available());close(pipefd[0]);
 puts("PASS: service stdin EOF, pipe data, unsigned bytes, closed pipe EOF");
}
''')
PY
"${CXX:-c++}" -std=c++11 "$work/test.cpp" -o "$work/test"
"$work/test"
