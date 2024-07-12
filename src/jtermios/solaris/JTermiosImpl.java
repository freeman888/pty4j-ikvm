/*
 * Copyright (c) 2011, Kustaa Nyholm / SpareTimeLabs
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list 
 * of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright notice, this 
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *  
 * Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its 
 * contributors may be used to endorse or promote products derived from this software 
 * without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */
package jtermios.solaris;

import java.io.File;

import java.nio.Buffer;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.regex.Pattern;

import jtermios.FDSet;

import jtermios.JTermios;
import jtermios.Pollfd;
import jtermios.Termios;
import jtermios.TimeVal;
import jtermios.solaris.JTermiosImpl.Solaris_C_lib.pollfd;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.NativeLong;
import com.sun.jna.Structure;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.NativeLongByReference;

import static jtermios.JTermios.*;
import static jtermios.JTermios.JTermiosLogging.log;

public class JTermiosImpl implements JTermiosInterface {

	private static String DEVICE_DIR_PATH = "/dev/term/";
	static Solaris_C_lib m_Clib = (Solaris_C_lib) Native.loadLibrary("c", Solaris_C_lib.class);

	public interface Solaris_C_lib extends com.sun.jna.Library {
		public int pipe(int[] fds);

		public int tcdrain(int fd);

		public int fcntl(int fd, int cmd, int[] arg);

		public int fcntl(int fd, int cmd, int arg);

		public int ioctl(int fd, int cmd, int[] arg);

		public int open(String path, int flags);

		public int close(int fd);

		public int tcgetattr(int fd, termios termios);

		public int tcsetattr(int fd, int cmd, termios termios);

		public int cfsetispeed(termios termios, NativeLong i);

		public int cfsetospeed(termios termios, NativeLong i);

		public NativeLong cfgetispeed(termios termios);

		public NativeLong cfgetospeed(termios termios);

		public NativeLong write(int fd, ByteBuffer buffer, NativeLong count);

		public NativeLong read(int fd, ByteBuffer buffer, NativeLong count);

		public int select(int n, int[] read, int[] write, int[] error, timeval timeout);

		public int poll(pollfd[] fds, int nfds, int timeout);
		
		public int poll(int[] fds, int nfds, int timeout);

		public int tcflush(int fd, int qs);

		public void perror(String msg);

		public int tcsendbreak(int fd, int duration);

		static public class timeval extends Structure {

			public NativeLong tv_sec;
			public NativeLong tv_usec;

			@Override
			protected List getFieldOrder() {
				return Arrays.asList(//
						"tv_sec",//
						"tv_usec"//
				);
			}

			public timeval(TimeVal timeout) {
				tv_sec = new NativeLong(timeout.tv_sec);
				tv_usec = new NativeLong(timeout.tv_usec);
			}
		}

		static public class pollfd extends Structure {

			public int fd;
			public short events;
			public short revents;

			@Override
			protected List getFieldOrder() {
				return Arrays.asList(//
						"fd",//
						"events",//
						"revents"//
				);
			}

			public pollfd(Pollfd pfd) {
				fd = pfd.fd;
				events = pfd.events;
				revents = pfd.revents;
			}
		}

		static public class termios extends Structure {

			public int c_iflag;
			public int c_oflag;
			public int c_cflag;
			public int c_lflag;
			public byte[] c_cc = new byte[32];

			@Override
			protected List getFieldOrder() {
				return Arrays.asList(//
						"c_iflag",//
						"c_oflag",//
						"c_cflag",//
						"c_lflag",//
						"c_cc"//
				);
			}

			public termios() {
			}

			public termios(Termios t) {
				c_iflag = t.c_iflag;
				c_oflag = t.c_oflag;
				c_cflag = t.c_cflag;
				c_lflag = t.c_lflag;
				System.arraycopy(t.c_cc, 0, c_cc, 0, t.c_cc.length);
			}

			public void update(Termios t) {
				t.c_iflag = c_iflag;
				t.c_oflag = c_oflag;
				t.c_cflag = c_cflag;
				t.c_lflag = c_lflag;
				System.arraycopy(c_cc, 0, t.c_cc, 0, t.c_cc.length);
			}
		}
	}

	static private class FDSetImpl extends FDSet {

		static final int FD_SET_SIZE = 1024;
		static final int NFBBITS = 32;
		int[] bits = new int[(FD_SET_SIZE + NFBBITS - 1) / NFBBITS];

		public String toString() {
			return String.format("%08X%08X", bits[0], bits[1]);
		}
	}

	public JTermiosImpl() {
		JTermiosLogging.log = JTermiosLogging.log && JTermiosLogging.log(1, "instantiating %s\n", getClass().getCanonicalName());

		// sys/filio.h stuff
		JTermios.FIONREAD = 0x4004667F;

		//fcntl.h stuff
		JTermios.O_RDWR = 0x00000002;
		JTermios.O_NONBLOCK = 0x00000080;
		JTermios.O_NOCTTY = 0x00000800;
		JTermios.O_NDELAY = 0x00000004;
		JTermios.F_GETFL = 0x00000003;
		JTermios.F_SETFL = 0x00000004;

		//errno.h stuff
		JTermios.EAGAIN = 11;
		JTermios.EBADF = 9;
		JTermios.EACCES = 22;
		JTermios.EEXIST = 17;
		JTermios.EINTR = 4;
		JTermios.EINVAL = 22;
		JTermios.EIO = 5;
		JTermios.EISDIR = 21;
		JTermios.ELOOP = 90;
		JTermios.EMFILE = 24;
		JTermios.ENAMETOOLONG = 78;
		JTermios.ENFILE = 23;
		JTermios.ENOENT = 2;
		JTermios.ENOSR = 63;
		JTermios.ENOSPC = 28;
		JTermios.ENOTDIR = 20;
		JTermios.ENXIO = 6;
		JTermios.EOVERFLOW = 79;
		JTermios.EROFS = 30;
		JTermios.ENOTSUP = 48;

		//termios.h stuff
		JTermios.TIOCM_RNG = 0x00000080;
		JTermios.TIOCM_CAR = 0x00000040;
		JTermios.IGNBRK = 0x00000001;
		JTermios.BRKINT = 0x00000002;
		JTermios.IGNPAR = 0x00000004;
		JTermios.PARMRK = 0x00000008;
		JTermios.INLCR = 0x00000040;
		JTermios.IGNCR = 0x00000080;
		JTermios.ICRNL = 0x00000100;
		JTermios.ECHONL = 0x00000040;
		JTermios.IEXTEN = 0x00008000;
		JTermios.CLOCAL = 0x00000800;
		JTermios.OPOST = 0x00000001;
		JTermios.VSTART = 0x00000008;
		JTermios.TCSANOW = 0x0000540E;
		JTermios.VSTOP = 0x00000009;
		JTermios.VMIN = 0x00000004;
		JTermios.VTIME = 0x00000005;
		JTermios.VEOF = 0x00000004;
		JTermios.TIOCMGET = 0x0000741D;
		JTermios.TIOCM_CTS = 0x00000020;
		JTermios.TIOCM_DSR = 0x00000100;
		JTermios.TIOCM_RI = 0x00000080;
		JTermios.TIOCM_CD = 0x00000040;
		JTermios.TIOCM_DTR = 0x00000002;
		JTermios.TIOCM_RTS = 0x00000004;
		JTermios.ICANON = 0x00000002;
		JTermios.ECHO = 0x00000008;
		JTermios.ECHOE = 0x00000010;
		JTermios.ISIG = 0x00000001;
		JTermios.TIOCMSET = 0x0000741A;
		JTermios.IXON = 0x00000400;
		JTermios.IXOFF = 0x00001000;
		JTermios.IXANY = 0x00000800;
		JTermios.CRTSCTS = 0x80000000;
		JTermios.TCSADRAIN = 0x0000540F;
		JTermios.INPCK = 0x00000010;
		JTermios.ISTRIP = 0x00000020;
		JTermios.CSIZE = 0x00000030;
		JTermios.TCIFLUSH = 0x00000000;
		JTermios.TCOFLUSH = 0x00000001;
		JTermios.TCIOFLUSH = 0x00000002;
		JTermios.CS5 = 0x00000000;
		JTermios.CS6 = 0x00000010;
		JTermios.CS7 = 0x00000020;
		JTermios.CS8 = 0x00000030;
		JTermios.CSTOPB = 0x00000040;
		JTermios.CREAD = 0x00000080;
		JTermios.PARENB = 0x00000100;
		JTermios.PARODD = 0x00000200;
		JTermios.B0 = 0;
		JTermios.B50 = 1;
		JTermios.B75 = 2;
		JTermios.B110 = 3;
		JTermios.B134 = 4;
		JTermios.B150 = 5;
		JTermios.B200 = 6;
		JTermios.B300 = 7;
		JTermios.B600 = 8;
		JTermios.B1200 = 8;
		JTermios.B1800 = 10;
		JTermios.B2400 = 11;
		JTermios.B4800 = 12;
		JTermios.B9600 = 13;
		JTermios.B19200 = 14;
		JTermios.B38400 = 15;
		JTermios.B57600 = 16;
		JTermios.B76800 = 17;
		JTermios.B115200 = 18;
		JTermios.B230400 = 20;

		//poll.h stuff
		JTermios.POLLIN = 0x0001;
		JTermios.POLLPRI = 0x0002;
		JTermios.POLLOUT = 0x0004;
		JTermios.POLLERR = 0x0008;
		JTermios.POLLNVAL = 0x0020;

		//select.h stuff

	}

	public int errno() {
		return Native.getLastError();
	}

	public void cfmakeraw(Termios termios) {
		Solaris_C_lib.termios t = new Solaris_C_lib.termios(termios);
		t.c_iflag &= ~(JTermios.IGNBRK | JTermios.BRKINT | JTermios.PARMRK | JTermios.ISTRIP | JTermios.INLCR | JTermios.IGNCR | JTermios.ICRNL | JTermios.IXON);
		t.c_oflag &= ~JTermios.OPOST;
		t.c_lflag &= ~(JTermios.ECHO | JTermios.ECHONL | JTermios.ICANON | JTermios.ISIG | JTermios.IEXTEN);
		t.c_cflag &= ~(JTermios.CSIZE | JTermios.PARENB);
		t.c_cflag |= JTermios.CS8;
		t.update(termios);
	}

	public int fcntl(int fd, int cmd, int[] arg) {
		return m_Clib.fcntl(fd, cmd, arg);
	}

	public int fcntl(int fd, int cmd, int arg) {
		return m_Clib.fcntl(fd, cmd, arg);
	}

	public int tcdrain(int fd) {
		return m_Clib.tcdrain(fd);
	}

	public int cfgetispeed(Termios termios) {
		return m_Clib.cfgetispeed(new Solaris_C_lib.termios(termios)).intValue();
	}

	public int cfgetospeed(Termios termios) {
		return m_Clib.cfgetospeed(new Solaris_C_lib.termios(termios)).intValue();
	}

	public int cfsetispeed(Termios termios, int speed) {
		Solaris_C_lib.termios t = new Solaris_C_lib.termios(termios);
		int ret = m_Clib.cfsetispeed(t, new NativeLong(speed));
		t.update(termios);
		return ret;
	}

	public int cfsetospeed(Termios termios, int speed) {
		Solaris_C_lib.termios t = new Solaris_C_lib.termios(termios);
		int ret = m_Clib.cfsetospeed(t, new NativeLong(speed));
		t.update(termios);
		return ret;
	}

	public int open(String s, int t) {
		if (s != null && !s.startsWith("/")) {
			s = DEVICE_DIR_PATH + s;
		}
		return m_Clib.open(s, t);
	}

	public int read(int fd, byte[] buffer, int len) {
		return m_Clib.read(fd, ByteBuffer.wrap(buffer), new NativeLong(len)).intValue();
	}

	public int write(int fd, byte[] buffer, int len) {
		return m_Clib.write(fd, ByteBuffer.wrap(buffer), new NativeLong(len)).intValue();
	}

	public int close(int fd) {
		return m_Clib.close(fd);
	}

	public int tcflush(int fd, int b) {
		return m_Clib.tcflush(fd, b);
	}

	public int tcgetattr(int fd, Termios termios) {
		Solaris_C_lib.termios t = new Solaris_C_lib.termios();
		int ret = m_Clib.tcgetattr(fd, t);
		t.update(termios);
		return ret;
	}

	public void perror(String msg) {
		m_Clib.perror(msg);
	}

	public int tcsendbreak(int fd, int duration) {
		// If duration is not zero, it sends zero-valued bits for duration*N seconds,
		// where N is at least 0.25, and not more than 0.5.
		return m_Clib.tcsendbreak(fd, duration / 250);
	}

	public int tcsetattr(int fd, int cmd, Termios termios) {
		return m_Clib.tcsetattr(fd, cmd, new Solaris_C_lib.termios(termios));
	}

	public void FD_CLR(int fd, FDSet set) {
		if (set == null) {
			return;
		}
		FDSetImpl p = (FDSetImpl) set;
		p.bits[fd / FDSetImpl.NFBBITS] &= ~(1 << (fd % FDSetImpl.NFBBITS));
	}

	public boolean FD_ISSET(int fd, FDSet set) {
		if (set == null) {
			return false;
		}
		FDSetImpl p = (FDSetImpl) set;
		return (p.bits[fd / FDSetImpl.NFBBITS] & (1 << (fd % FDSetImpl.NFBBITS))) != 0;
	}

	public void FD_SET(int fd, FDSet set) {
		if (set == null) {
			return;
		}
		FDSetImpl p = (FDSetImpl) set;
		p.bits[fd / FDSetImpl.NFBBITS] |= 1 << (fd % FDSetImpl.NFBBITS);
	}

	public void FD_ZERO(FDSet set) {
		if (set == null) {
			return;
		}
		FDSetImpl p = (FDSetImpl) set;
		Arrays.fill(p.bits, 0);
	}

	public int select(int nfds, FDSet rfds, FDSet wfds, FDSet efds, TimeVal timeout) {
		Solaris_C_lib.timeval tout = null;
		if (timeout != null) {
			tout = new Solaris_C_lib.timeval(timeout);
		}

		int[] r = rfds != null ? ((FDSetImpl) rfds).bits : null;
		int[] w = wfds != null ? ((FDSetImpl) wfds).bits : null;
		int[] e = efds != null ? ((FDSetImpl) efds).bits : null;
		return m_Clib.select(nfds, r, w, e, tout);
	}

	public int poll(Pollfd fds[], int nfds, int timeout) {
		pollfd[] pfds = new pollfd[fds.length];
		for (int i = 0; i < nfds; i++)
			pfds[i] = new pollfd(fds[i]);
		int ret = m_Clib.poll(pfds, nfds, timeout);
		for (int i = 0; i < nfds; i++)
			fds[i].revents = pfds[i].revents;
		return ret;
	}
	
	public int poll(int fds[], int nfds, int timeout) {
		return m_Clib.poll(fds, nfds, timeout);
	}

	public FDSet newFDSet() {
		return new FDSetImpl();
	}

	public int ioctl(int fd, int cmd, int[] data) {
		return m_Clib.ioctl(fd, cmd, data);
	}

	public String getPortNamePattern() {
		return ".*";
	}

	public List<String> getPortList() {
		File dir = new File(DEVICE_DIR_PATH);
		if (!dir.isDirectory()) {
			JTermiosLogging.log = JTermiosLogging.log && JTermiosLogging.log(1, "device directory %s does not exist\n", DEVICE_DIR_PATH);
			return null;
		}
		String[] devs = dir.list();
		LinkedList<String> list = new LinkedList<String>();

		Pattern p = JTermios.getPortNamePattern(this);
		if (devs != null) {
			for (int i = 0; i < devs.length; i++) {
				String s = devs[i];
				if (p.matcher(s).matches())
					list.add(s);
			}
		}
		return list;
	}

	public void shutDown() {
	}

	public int setspeed(int fd, Termios termios, int speed) {
		int br = speed;
		switch (speed) {
			case 50:
				br = JTermios.B50;
				break;
			case 75:
				br = JTermios.B75;
				break;
			case 110:
				br = JTermios.B110;
				break;
			case 134:
				br = JTermios.B134;
				break;
			case 150:
				br = JTermios.B150;
				break;
			case 200:
				br = JTermios.B200;
				break;
			case 300:
				br = JTermios.B300;
				break;
			case 600:
				br = JTermios.B600;
				break;
			case 1200:
				br = JTermios.B1200;
				break;
			case 1800:
				br = JTermios.B1800;
				break;
			case 2400:
				br = JTermios.B2400;
				break;
			case 4800:
				br = JTermios.B4800;
				break;
			case 9600:
				br = JTermios.B9600;
				break;
			case 19200:
				br = JTermios.B19200;
				break;
			case 38400:
				br = JTermios.B38400;
				break;
			case 7200:
				br = JTermios.B7200;
				break;
			case 14400:
				br = JTermios.B14400;
				break;
			case 28800:
				br = JTermios.B28800;
				break;
			case 57600:
				br = JTermios.B57600;
				break;
			case 76800:
				br = JTermios.B76800;
				break;
			case 115200:
				br = JTermios.B115200;
				break;
			case 230400:
				br = JTermios.B230400;
				break;
		}
		int r;
		if ((r = cfsetispeed(termios, br)) != 0)
			return r;
		if ((r = cfsetospeed(termios, br)) != 0)
			return r;
		if ((r = tcsetattr(fd, JTermios.TCSANOW, termios)) != 0)
			return r;
		return 0;
	}
	
	public int pipe(int[] fds) {
		return m_Clib.pipe(fds);
	}

}
