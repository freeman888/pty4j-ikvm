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

package jtermios.linux;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.io.IOException;

import java.lang.reflect.Method;
import java.nio.Buffer;

import java.nio.ByteBuffer;
import java.util.*;
import java.util.regex.Pattern;

import com.sun.jna.*;
import jtermios.*;

import jtermios.JTermios.JTermiosInterface;
import jtermios.linux.JTermiosImpl.Linux_C_lib.pollfd;
import jtermios.linux.JTermiosImpl.Linux_C_lib.serial_struct;

import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.NativeLongByReference;

import static jtermios.JTermios.*;
import static jtermios.JTermios.JTermiosLogging.log;

public class JTermiosImpl implements JTermiosInterface {
	private static String DEVICE_DIR_PATH = "/dev/";
	private static final boolean IS64B = NativeLong.SIZE == 8;
	static Linux_C_lib_DirectMapping m_ClibDM = new Linux_C_lib_DirectMapping();
	static Linux_C_lib m_Clib = m_ClibDM;

	private final static int TIOCGSERIAL = 0x0000541E;
	private final static int TIOCSSERIAL = 0x0000541F;

	private final static int ASYNC_SPD_MASK = 0x00001030;
	private final static int ASYNC_SPD_CUST = 0x00000030;

	private final static int[] m_BaudRates = { //
	50, 0000001, //
			75, 0000002, //
			110, 0000003, //
			134, 0000004, //
			150, 0000005, //
			200, 0000006, //
			300, 0000007, //
			600, 0000010, //
			1200, 0000011, //
			1800, 0000012, //
			2400, 0000013, //
			4800, 0000014, //
			9600, 0000015, //
			19200, 0000016, //
			38400, 0000017, //
			57600, 0010001, //
			115200, 0010002, //
			230400, 0010003, //
			460800, 0010004, //
			500000, 0010005, //
			576000, 0010006, //
			921600, 0010007, //
			1000000, 0010010, //
			1152000, 0010011, //
			1500000, 0010012, //
			2000000, 0010013, //
			2500000, 0010014, //
			3000000, 0010015, //
			3500000, 0010016, //
			4000000, 0010017 //
	};

	public static class Linux_C_lib_DirectMapping implements Linux_C_lib {
		native public long memcpy(int[] dst, short[] src, long n);

		native public int memcpy(int[] dst, short[] src, int n);

		native public int pipe(int[] fds);

		native public int tcdrain(int fd);

		native public void cfmakeraw(termios termios);

		native public int fcntl(int fd, int cmd, int arg);

		native public int ioctl(int fd, int cmd, int[] arg);

		native public int ioctl(int fd, int cmd, serial_struct arg);

		native public int open(String path, int flags);

		native public int close(int fd);

		native public int tcgetattr(int fd, termios termios);

		native public int tcsetattr(int fd, int cmd, termios termios);

		native public int cfsetispeed(termios termios, NativeLong i);

		native public int cfsetospeed(termios termios, NativeLong i);

		native public NativeLong cfgetispeed(termios termios);

		native public NativeLong cfgetospeed(termios termios);

		native public int write(int fd, byte[] buffer, int count);

		native public int read(int fd, byte[] buffer, int count);

		native public long write(int fd, byte[] buffer, long count);

		native public long read(int fd, byte[] buffer, long count);

		native public int select(int n, int[] read, int[] write, int[] error, timeval timeout);

		native public int poll(int[] fds, int nfds, int timeout);

		public int poll(pollfd[] fds, int nfds, int timeout) {
			throw new IllegalArgumentException();
		}

		native public int tcflush(int fd, int qs);

		native public void perror(String msg);

		native public int tcsendbreak(int fd, int duration);

		static {
			try {
				Native.register(NativeLibrary.getInstance("c", YJPFunctionMapper.OPTIONS));
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	public interface Linux_C_lib extends Library {
		public long memcpy(int[] dst, short[] src, long n);

		public int memcpy(int[] dst, short[] src, int n);

		public int pipe(int[] fds);

		public int tcdrain(int fd);

		public void cfmakeraw(termios termios);

		public int fcntl(int fd, int cmd, int arg);

		public int ioctl(int fd, int cmd, int[] arg);

		public int ioctl(int fd, int cmd, serial_struct arg);

		public int open(String path, int flags);

		public int close(int fd);

		public int tcgetattr(int fd, termios termios);

		public int tcsetattr(int fd, int cmd, termios termios);

		public int cfsetispeed(termios termios, NativeLong i);

		public int cfsetospeed(termios termios, NativeLong i);

		public NativeLong cfgetispeed(termios termios);

		public NativeLong cfgetospeed(termios termios);

		public int write(int fd, byte[] buffer, int count);

		public int read(int fd, byte[] buffer, int count);

		public long write(int fd, byte[] buffer, long count);

		public long read(int fd, byte[] buffer, long count);

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

		public static class serial_struct extends Structure {
			public int type;
			public int line;
			public int port;
			public int irq;
			public int flags;
			public int xmit_fifo_size;
			public int custom_divisor;
			public int baud_base;
			public short close_delay;
			public short io_type;
			//public char io_type;
			//public char reserved_char;
			public int hub6;
			public short closing_wait;
			public short closing_wait2;
			public Pointer iomem_base;
			public short iomem_reg_shift;
			public int port_high;
			public NativeLong iomap_base;

			@Override
			protected List getFieldOrder() {
				return Arrays.asList(//
						"type",//
						"line",//
						"port",//
						"irq",//
						"flags",//
						"xmit_fifo_size",//
						"custom_divisor",//
						"baud_base",//
						"close_delay",//
						"io_type",//
						//public char io_type;
						//public char reserved_char;
						"hub6",//
						"closing_wait",//
						"closing_wait2",//
						"iomem_base",//
						"iomem_reg_shift",//
						"port_high",//
						"iomap_base"//
				);
			}
		};

		static public class termios extends Structure {
			public int c_iflag;
			public int c_oflag;
			public int c_cflag;
			public int c_lflag;
			public byte c_line;
			public byte[] c_cc = new byte[32];
			public int c_ispeed;
			public int c_ospeed;

			@Override
			protected List getFieldOrder() {
				return Arrays.asList(//
						"c_iflag",//
						"c_oflag",//
						"c_cflag",//
						"c_lflag",//
						"c_line",//
						"c_cc",//
						"c_ispeed",//
						"c_ospeed"//
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
				c_ispeed = t.c_ispeed;
				c_ospeed = t.c_ospeed;
			}

			public void update(Termios t) {
				t.c_iflag = c_iflag;
				t.c_oflag = c_oflag;
				t.c_cflag = c_cflag;
				t.c_lflag = c_lflag;
				System.arraycopy(c_cc, 0, t.c_cc, 0, t.c_cc.length);
				t.c_ispeed = c_ispeed;
				t.c_ospeed = c_ospeed;
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

		//linux/serial.h stuff
		JTermios.FIONREAD = 0x541B; // Looked up manually
		//fcntl.h stuff
		JTermios.O_RDWR = 0x00000002;
		JTermios.O_NONBLOCK = 0x00000800;
		JTermios.O_NOCTTY = 0x00000100;
		JTermios.O_NDELAY = 0x00000800;
		JTermios.F_GETFL = 0x00000003;
		JTermios.F_SETFL = 0x00000004;
		//errno.h stuff
		JTermios.EAGAIN = 11;
		JTermios.EACCES = 13;
		JTermios.EEXIST = 17;
		JTermios.EINTR = 4;
		JTermios.EINVAL = 22;
		JTermios.EIO = 5;
		JTermios.EISDIR = 21;
		JTermios.ELOOP = 40;
		JTermios.EMFILE = 24;
		JTermios.ENAMETOOLONG = 36;
		JTermios.ENFILE = 23;
		JTermios.ENOENT = 2;
		JTermios.ENOSR = 63;
		JTermios.ENOSPC = 28;
		JTermios.ENOTDIR = 20;
		JTermios.ENXIO = 6;
		JTermios.EOVERFLOW = 75;
		JTermios.EROFS = 30;
		JTermios.ENOTSUP = 95;
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
		JTermios.TCSANOW = 0x00000000;
		JTermios.VSTOP = 0x00000009;
		JTermios.VMIN = 0x00000006;
		JTermios.VTIME = 0x00000005;
		JTermios.VEOF = 0x00000004;
		JTermios.TIOCMGET = 0x00005415;
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
		JTermios.TIOCMSET = 0x00005418;
		JTermios.IXON = 0x00000400;
		JTermios.IXOFF = 0x00001000;
		JTermios.IXANY = 0x00000800;
		JTermios.CRTSCTS = 0x80000000;
		JTermios.TCSADRAIN = 0x00000001;
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
		JTermios.B1200 = 9;
		JTermios.B1800 = 10;
		JTermios.B2400 = 11;
		JTermios.B4800 = 12;
		JTermios.B9600 = 13;
		JTermios.B19200 = 14;
		JTermios.B38400 = 15;
		JTermios.B57600 = 4097;
		JTermios.B115200 = 4098;
		JTermios.B230400 = 4099;
		//poll.h stuff
		JTermios.POLLIN = 0x0001;
		JTermios.POLLPRI = 0x0002;
		JTermios.POLLOUT = 0x0004;
		JTermios.POLLERR = 0x0008;
		JTermios.POLLNVAL = 0x0020;

		// these depend on the endianness off the machine
		JTermios.POLLIN_IN = pollMask(0, JTermios.POLLIN);
		JTermios.POLLOUT_IN = pollMask(0, JTermios.POLLOUT);

		JTermios.POLLIN_OUT = pollMask(1, JTermios.POLLIN);
		JTermios.POLLOUT_OUT = pollMask(1, JTermios.POLLOUT);
		JTermios.POLLNVAL_OUT = pollMask(1, JTermios.POLLNVAL);
	}

	public static int pollMask(int i, short n) {
		short[] s = new short[2];
		int[] d = new int[1];
		s[i] = n;
		// the native call depends on weather this is 32 or 64 bit arc
		if (IS64B)
			m_ClibDM.memcpy(d, s, (long) 4);
		else
			m_ClibDM.memcpy(d, s, (int) 4);
		return d[0];
	}

	public int errno() {
		return Native.getLastError();
	}

	public void cfmakeraw(Termios termios) {
		Linux_C_lib.termios t = new Linux_C_lib.termios(termios);
		m_Clib.cfmakeraw(t);
		t.update(termios);
	}

	public int fcntl(int fd, int cmd, int arg) {
		return m_Clib.fcntl(fd, cmd, arg);
	}

	public int tcdrain(int fd) {
		return m_Clib.tcdrain(fd);
	}

	public int cfgetispeed(Termios termios) {
		return m_Clib.cfgetispeed(new Linux_C_lib.termios(termios)).intValue();
	}

	public int cfgetospeed(Termios termios) {
		return m_Clib.cfgetospeed(new Linux_C_lib.termios(termios)).intValue();
	}

	public int cfsetispeed(Termios termios, int speed) {
		Linux_C_lib.termios t = new Linux_C_lib.termios(termios);
		int ret = m_Clib.cfsetispeed(t, new NativeLong(speed));
		t.update(termios);
		return ret;
	}

	public int cfsetospeed(Termios termios, int speed) {
		Linux_C_lib.termios t = new Linux_C_lib.termios(termios);
		int ret = m_Clib.cfsetospeed(t, new NativeLong(speed));
		t.update(termios);
		return ret;
	}

	public int open(String s, int t) {
		if (s != null && !s.startsWith("/"))
			s = DEVICE_DIR_PATH + s;
		return m_Clib.open(s, t);
	}

	public int read(int fd, byte[] buffer, int len) {
		if (IS64B)
			return (int) m_Clib.read(fd, buffer, (long) len);
		else
			return m_Clib.read(fd, buffer, len);
	}

	public int write(int fd, byte[] buffer, int len) {
		if (IS64B)
			return (int) m_Clib.write(fd, buffer, (long) len);
		else
			return m_Clib.write(fd, buffer, len);
	}

	public int close(int fd) {
		return m_Clib.close(fd);
	}

	public int tcflush(int fd, int b) {
		return m_Clib.tcflush(fd, b);
	}

	public int tcgetattr(int fd, Termios termios) {
		Linux_C_lib.termios t = new Linux_C_lib.termios();
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
		return m_Clib.tcsetattr(fd, cmd, new Linux_C_lib.termios(termios));
	}

	public void FD_CLR(int fd, FDSet set) {
		if (set == null)
			return;
		FDSetImpl p = (FDSetImpl) set;
		p.bits[fd / FDSetImpl.NFBBITS] &= ~(1 << (fd % FDSetImpl.NFBBITS));
	}

	public boolean FD_ISSET(int fd, FDSet set) {
		if (set == null)
			return false;
		FDSetImpl p = (FDSetImpl) set;
		return (p.bits[fd / FDSetImpl.NFBBITS] & (1 << (fd % FDSetImpl.NFBBITS))) != 0;
	}

	public void FD_SET(int fd, FDSet set) {
		if (set == null)
			return;
		FDSetImpl p = (FDSetImpl) set;
		p.bits[fd / FDSetImpl.NFBBITS] |= 1 << (fd % FDSetImpl.NFBBITS);
	}

	public void FD_ZERO(FDSet set) {
		if (set == null)
			return;
		FDSetImpl p = (FDSetImpl) set;
		Arrays.fill(p.bits, 0);
	}

	public int select(int nfds, FDSet rfds, FDSet wfds, FDSet efds, TimeVal timeout) {
		Linux_C_lib.timeval tout = null;
		if (timeout != null)
			tout = new Linux_C_lib.timeval(timeout);

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

	// This ioctl is Linux specific, so keep it private for now
	private int ioctl(int fd, int cmd, serial_struct data) {
		// Do the logging here as this does not go through the JTermios which normally does the logging
		JTermiosLogging.log = JTermiosLogging.log && JTermiosLogging.log(5, "> ioctl(%d,%d,%s)\n", fd, cmd, data);
		int ret = m_Clib.ioctl(fd, cmd, data);
		JTermiosLogging.log = JTermiosLogging.log && JTermiosLogging.log(3, "< tcsetattr(%d,%d,%s) => %d\n", fd, cmd, data, ret);
		return ret;
	}

	public String getPortNamePattern() {
		// First we have to determine which serial drivers exist and which
		// prefixes they use
		final List<String> prefixes = new ArrayList<String>();

		try {
			BufferedReader drivers = new BufferedReader(new InputStreamReader(new FileInputStream("/proc/tty/drivers"), "US-ASCII"));
			String line;
			while ((line = drivers.readLine()) != null) {
				// /proc/tty/drivers contains the prefix in the second column
				// and "serial" in the fifth

				String[] parts = line.split(" +");
				if (parts.length != 5) {
					continue;
				}

				if (!"serial".equals(parts[4])) {
					continue;
				}

				// Sanity check the prefix
				if (!parts[1].startsWith("/dev/")) {
					continue;
				}

				prefixes.add(parts[1].substring(5));
			}
			drivers.close();
		} catch (IOException e) {
			JTermiosLogging.log = JTermiosLogging.log && JTermiosLogging.log(1, "failed to read /proc/tty/drivers\n");

			prefixes.add("ttyS");
			prefixes.add("ttyUSB");
			prefixes.add("ttyACM");
		}

		// Now build the pattern from the known prefixes

		StringBuilder pattern = new StringBuilder();

		pattern.append('^');

		boolean first = true;
		for (String prefix : prefixes) {
			if (first) {
				first = false;
			} else {
				pattern.append('|');
			}

			pattern.append("(");
			pattern.append(prefix);
			pattern.append(".+)");
		}

		return pattern.toString();
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
		int c = speed;
		int r;
		for (int i = 0; i < m_BaudRates.length; i += 2) {
			if (m_BaudRates[i] == speed) {

				// found the baudrate from the table

				// just in case custom divisor was in use, try to turn it off first
				serial_struct ss = new serial_struct();

				r = ioctl(fd, TIOCGSERIAL, ss);
				if (r == 0) {
					ss.flags &= ~ASYNC_SPD_MASK;
					r = ioctl(fd, TIOCSSERIAL, ss);
				}

				// now set the speed with the constant from the table
				c = m_BaudRates[i + 1];
				if ((r = JTermios.cfsetispeed(termios, c)) != 0)
					return r;
				if ((r = JTermios.cfsetospeed(termios, c)) != 0)
					return r;
				if ((r = JTermios.tcsetattr(fd, JTermios.TCSANOW, termios)) != 0)
					return r;

				return 0;
			}
		}

		// baudrate not defined in the table, try custom divisor approach

		// configure port to use custom speed instead of 38400
		serial_struct ss = new serial_struct();
		if ((r = ioctl(fd, TIOCGSERIAL, ss)) != 0)
			return r;
		ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;

		if (speed == 0) {
			JTermiosLogging.log = JTermiosLogging.log && JTermiosLogging.log(1, "unable to set custom baudrate %d \n", speed);
			return -1;
		}

		ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;

		if (ss.custom_divisor == 0) {
			JTermiosLogging.log = JTermiosLogging.log && JTermiosLogging.log(1, "unable to set custom baudrate %d (possible division by zero)\n", speed);
			return -1;
		}

		int closestSpeed = ss.baud_base / ss.custom_divisor;

		if (closestSpeed < speed * 98 / 100 || closestSpeed > speed * 102 / 100) {
			JTermiosLogging.log = JTermiosLogging.log && JTermiosLogging.log(1, "best available baudrate %d not close enough to requested %d \n", closestSpeed, speed);
			return -1;
		}

		if ((r = ioctl(fd, TIOCSSERIAL, ss)) != 0)
			return r;

		if ((r = JTermios.cfsetispeed(termios, JTermios.B38400)) != 0)
			return r;
		if ((r = JTermios.cfsetospeed(termios, JTermios.B38400)) != 0)
			return r;
		if ((r = JTermios.tcsetattr(fd, JTermios.TCSANOW, termios)) != 0)
			return r;
		return 0;
	}

	public int pipe(int[] fds) {
		return m_Clib.pipe(fds);
	}
}
