package jtermios;

import com.sun.jna.FunctionMapper;
import com.sun.jna.Library;
import com.sun.jna.NativeLibrary;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

/**
 * FunctionMapping to strip off YourKit Profiler function prefixes.
 * Serves as a workaround to a conflict between YourKit and JNA direct calls.
 *
 * For details see: https://github.com/twall/jna/issues/236
 *
 * @author traff
 */
public class YJPFunctionMapper implements FunctionMapper {
    public final static Map OPTIONS = new HashMap();

    static {
        OPTIONS.put(Library.OPTION_FUNCTION_MAPPER, new YJPFunctionMapper());
    }

    private static final String YJP_PREFIX = "$$YJP$$";

    @Override
    public String getFunctionName(NativeLibrary nativeLibrary, Method method) {
        if (method.getName().startsWith(YJP_PREFIX)) {
            return method.getName().substring(YJP_PREFIX.length());
        } else {
            return method.getName();
        }
    }
}
