// frc/lib/logging/ExtendedLogger.java
package frc.lib.logging;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.RecordComponent;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicInteger;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.LongConsumer;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Notifier;

import dev.doglog.DogLog;

/**
 * Logger qui wrap DogLog, avec auto-logging via annotation + cache.
 *
 * - @LoggableField - log auto (plan compilé 1x)
 * - @TunableField - tunable auto (callback met à jour ton field)
 * - Support: primitives, boxed, arrays primitives, String, enums, Measure, records (explosés).
 *
 * Usage:
 * - Dans tes constructors (subsystems, etc): ExtendedLogger.registerInstance(this)
 * - Une fois (robotInit): ExtendedLogger.startBackground(20) ex 20Hz
 */
public final class ExtendedLogger {

    // =========================
    // Config
    // =========================

    /** Anti-spam erreurs (en ticks logger). */
    private static final int FAULT_COOLDOWN_TICKS = 50;

    /** Default si startBackground jamais appelé. */
    private static final double DEFAULT_BG_HZ = 50.0;

    // =========================
    // Runtime state (thread-safe)
    // =========================

    /**
     * FIX: liste parallèle compilée à l'enregistrement.
     * Hot loop = zéro allocations + zéro map lookups.
     */
    private static final CopyOnWriteArrayList<Registered> registered = new CopyOnWriteArrayList<>();

    /** Pour éviter d'enregistrer 2 fois le même objet (identité). */
    private static final Set<IdKey> registeredSet = ConcurrentHashMap.newKeySet();

    /** Tunables déjà bindés (instance - fields set) pour éviter double bind. */
    private static final Map<IdKey, Set<Field>> boundTunables = new ConcurrentHashMap<>();

    /** Cache des plans de record (par classe). */
    private static final Map<Class<?>, RecordPlan> recordPlans = new ConcurrentHashMap<>();

    /** Anti-spam erreurs: key - last tick logged. */
    private static final Map<String, Integer> faultCooldown = new ConcurrentHashMap<>();

    /** Tick counter utilisé pour decimation + cooldown. */
    private static final AtomicInteger tickCounter = new AtomicInteger(0);

    // =========================
    // Background Notifier
    // =========================

    private static volatile boolean bgStarted = false;
    private static volatile double bgHz = DEFAULT_BG_HZ;
    private static Notifier bgNotifier = null;

    public static synchronized void startBackground(double hz) {
        if (bgStarted)
            return;

        if (!(hz > 0.0))
            hz = 10.0;
        bgHz = hz;

        bgNotifier = new Notifier(() -> {
            try {
                runOnce();
            } catch (Throwable t) {
                DogLog.logFault("ExtendedLogger background crash: " +
                        t.getClass().getSimpleName() + " " + t.getMessage());
            }
        });

        bgNotifier.setName("ExtendedLogger");
        bgNotifier.startPeriodic(1.0 / hz);
        bgStarted = true;
    }

    public static synchronized void stopBackground() {
        if (!bgStarted)
            return;
        try {
            if (bgNotifier != null)
                bgNotifier.stop();
        } finally {
            bgNotifier = null;
            bgStarted = false;
            bgHz = DEFAULT_BG_HZ;
        }
    }

    // =========================
    // Handlers (compile-time chosen)
    // =========================

    /**
     * Map "declared type - handler exact".
     * IMPORTANT: on évite les scans assignable en hot-loop.
     */
    private static final Map<Class<?>, BiConsumer<String, Object>> handlers = new ConcurrentHashMap<>();

    static {
        // --- primitives (Field.get returns boxed anyway)
        handlers.put(boolean.class, (p, o) -> DogLog.log(p, ((Boolean) o).booleanValue()));
        handlers.put(double.class, (p, o) -> DogLog.log(p, ((Number) o).doubleValue()));
        handlers.put(float.class, (p, o) -> DogLog.log(p, ((Number) o).floatValue()));
        handlers.put(int.class, (p, o) -> DogLog.log(p, ((Number) o).longValue())); // DogLog uses long
        handlers.put(long.class, (p, o) -> DogLog.log(p, ((Number) o).longValue()));

        // --- boxed
        handlers.put(Boolean.class, (p, o) -> DogLog.log(p, (Boolean) o));
        handlers.put(Double.class, (p, o) -> DogLog.log(p, (Double) o));
        handlers.put(Float.class, (p, o) -> DogLog.log(p, (Float) o));
        handlers.put(Integer.class, (p, o) -> DogLog.log(p, ((Integer) o).longValue()));
        handlers.put(Long.class, (p, o) -> DogLog.log(p, (Long) o));
        handlers.put(String.class, (p, o) -> DogLog.log(p, (String) o));

        // --- measures
        handlers.put(Measure.class, (p, o) -> DogLog.log(p, (Measure<?>) o));

        // --- arrays primitives
        handlers.put(boolean[].class, (p, o) -> DogLog.log(p, (boolean[]) o));
        handlers.put(double[].class, (p, o) -> DogLog.log(p, (double[]) o));
        handlers.put(float[].class, (p, o) -> DogLog.log(p, (float[]) o));
        handlers.put(int[].class, (p, o) -> DogLog.log(p, (int[]) o));
        handlers.put(long[].class, (p, o) -> DogLog.log(p, (long[]) o));
        handlers.put(String[].class, (p, o) -> DogLog.log(p, (String[]) o));

        // --- records explode
        handlers.put(Record.class, (p, o) -> logRecord(p, (Record) o));
    }

    // =========================
    // Public API
    // =========================

    /**
     * Enregistre une instance (ex: subsystem) pour auto-logging + auto-tunable.
     * Compile un plan 1x.
     */
    public static void registerInstance(Object instance) {
        if (instance == null)
            return;

        // allocate key ONCE at registration time
        IdKey key = IdKey.of(instance);

        // idempotent identity register
        if (!registeredSet.add(key))
            return;

        // Bind tunables first (so they exist immediately)
        bindTunables(key, instance);

        // compile plan ONCE and store it alongside (instance, key)
        List<LogAction> plan = buildPlan(instance);
        registered.add(new Registered(instance, key, plan));
    }

    // =========================
    // Tick
    // =========================

    private static void runOnce() {
        int tick = tickCounter.incrementAndGet();
        logAll(tick);
    }

    private static void logAll(int tick) {
        // NO allocations, NO map lookups
        for (Registered r : registered) {
            final Object inst = r.instance;
            final List<LogAction> actions = r.plan;

            for (LogAction a : actions) {
                if (!a.shouldRun(tick))
                    continue;

                try {
                    a.run(inst);
                } catch (IllegalAccessException e) {
                    logFaultThrottled(
                            "illegalAccess:" + inst.getClass().getName() + "." + a.field.getName(),
                            "Accès refusé au field " + a.field.getName(),
                            tick);
                } catch (Exception e) {
                    logFaultThrottled(
                            "other:" + inst.getClass().getName() + "." + a.field.getName(),
                            "Erreur logging " + a.field.getName() + ": " +
                                    e.getClass().getSimpleName() + " " + e.getMessage(),
                            tick);
                }
            }
        }
    }

    // =========================
    // Registered entry (fix)
    // =========================

    private static final class Registered {
        final Object instance;
        @SuppressWarnings("unused")
        final IdKey key; // kept for debugging / future removal : not used in hot loop
        final List<LogAction> plan;

        Registered(Object instance, IdKey key, List<LogAction> plan) {
            this.instance = instance;
            this.key = key;
            this.plan = plan;
        }
    }

    // =========================
    // Plan compilation
    // =========================

    private static final class LogAction {
        final String path;
        final Field field;
        final BiConsumer<String, Object> handler;
        final int periodTicks;

        LogAction(String path, Field field, BiConsumer<String, Object> handler, int periodTicks) {
            this.path = path;
            this.field = field;
            this.handler = handler;
            this.periodTicks = Math.max(1, periodTicks);
        }

        boolean shouldRun(int tick) {
            return (tick % periodTicks) == 0;
        }

        void run(Object inst) throws IllegalAccessException {
            Object v = field.get(inst);
            if (v == null)
                return;
            handler.accept(path, v);
        }
    }

    private static List<LogAction> buildPlan(Object instance) {
        List<LogAction> actions = new ArrayList<>();

        for (Field f : getAllFields(instance.getClass())) {
            LoggableField ann = f.getAnnotation(LoggableField.class);
            if (ann == null)
                continue;

            try {
                f.setAccessible(true);
            } catch (Exception ignored) {
            }

            String path = resolveLogPath(instance, f, ann.path());
            int periodTicks = hzToPeriodTicks(ann.hz());

            Class<?> declared = f.getType();
            BiConsumer<String, Object> h = handlers.get(declared);

            // Enums: DogLog doesn't log Enum directly - log String
            if (h == null && declared.isEnum()) {
                h = (p, o) -> DogLog.log(p, ((Enum<?>) o).name());
            }

            // Interface/supertype fallback
            if (h == null && Measure.class.isAssignableFrom(declared))
                h = handlers.get(Measure.class);
            if (h == null && Record.class.isAssignableFrom(declared))
                h = handlers.get(Record.class);

            // Fallback safe
            if (h == null) {
                h = (p, o) -> DogLog.log(p, String.valueOf(o));
            }

            actions.add(new LogAction(path, f, h, periodTicks));
        }

        return actions;
    }

    private static String resolveLogPath(Object instance, Field f, String annotatedPath) {
        if (annotatedPath != null && !annotatedPath.isBlank())
            return annotatedPath;
        return instance.getClass().getSimpleName() + "/" + f.getName();
    }

    /**
     * Convertit un hz demandé en periodTicks basé sur le background loop réel.
     * Si startBackground() n'est pas démarré, on assume DEFAULT_BG_HZ.
     */
    private static int hzToPeriodTicks(double requestedHz) {
        double loopHz = bgHz;
        if (!(loopHz > 0.0))
            loopHz = DEFAULT_BG_HZ;

        if (!(requestedHz > 0.0))
            return 5;
        if (requestedHz >= loopHz)
            return 1;

        int p = (int) Math.round(loopHz / requestedHz);
        return Math.max(1, p);
    }

    private static List<Field> getAllFields(Class<?> type) {
        ArrayList<Field> out = new ArrayList<>();
        for (Class<?> c = type; c != null && c != Object.class; c = c.getSuperclass()) {
            Field[] fs = c.getDeclaredFields();
            for (Field f : fs)
                out.add(f);
        }
        return out;
    }

    // =========================
    // Tunables
    // =========================

    private static void bindTunables(IdKey key, Object instance) {
        // Thread-safe set, avoids IdentityHashMap races.
        Set<Field> already = boundTunables.computeIfAbsent(key, k -> ConcurrentHashMap.newKeySet());

        for (Field f : getAllFields(instance.getClass())) {
            TunableField ann = f.getAnnotation(TunableField.class);
            if (ann == null)
                continue;
            if (!already.add(f))
                continue; // atomic: add returns false if already present

            try {
                f.setAccessible(true);
            } catch (Exception ignored) {
            }

            String tKey = resolveTunableKey(instance, f, ann.key());
            String unit = (ann.unit() == null) ? "" : ann.unit();

            Class<?> t = f.getType();

            try {
                // double / Double
                if (t == double.class || t == Double.class) {
                    double def = (t == double.class) ? f.getDouble(instance) : (Double) f.get(instance);
                    def = clamp(def, ann.min(), ann.max());

                    DoubleConsumer cb = (v -> setDouble(instance, f, clamp(v, ann.min(), ann.max())));
                    tunableDouble(tKey, def, unit, cb);
                    continue;
                }

                // float / Float (double tunable, then cast)
                if (t == float.class || t == Float.class) {
                    float defF = (t == float.class) ? f.getFloat(instance) : (Float) f.get(instance);
                    double def = clamp(defF, ann.min(), ann.max());

                    DoubleConsumer cb = (v -> setFloat(instance, f, (float) clamp(v, ann.min(), ann.max())));
                    tunableDouble(tKey, def, unit, cb);
                    continue;
                }

                // long / Long
                if (t == long.class || t == Long.class) {
                    long def = (t == long.class) ? f.getLong(instance) : (Long) f.get(instance);

                    LongConsumer cb = (v -> setLong(instance, f, v));
                    tunableLong(tKey, def, unit, cb);
                    continue;
                }

                // int / Integer (mapped to long tunable)
                if (t == int.class || t == Integer.class) {
                    int defI = (t == int.class) ? f.getInt(instance) : (Integer) f.get(instance);
                    long def = defI;

                    LongConsumer cb = (v -> setInt(instance, f, (int) v));
                    tunableLong(tKey, def, unit, cb);
                    continue;
                }

                // boolean / Boolean
                if (t == boolean.class || t == Boolean.class) {
                    boolean def = (t == boolean.class) ? f.getBoolean(instance) : (Boolean) f.get(instance);

                    BooleanConsumer cb = (v -> setBoolean(instance, f, v));
                    tunableBoolean(tKey, def, unit, cb);
                    continue;
                }

                // String
                if (t == String.class) {
                    String def = (String) f.get(instance);
                    if (def == null)
                        def = "";

                    Consumer<String> cb = (v -> setObject(instance, f, v));
                    tunableString(tKey, def, unit, cb);
                    continue;
                }

                // Enum (stored as String tunable)
                if (t.isEnum()) {
                    Enum<?> defE = (Enum<?>) f.get(instance);
                    String defName = (defE != null) ? defE.name() : "";

                    Consumer<String> cb = (v -> setEnumFromName(instance, f, v));
                    tunableString(tKey, defName, unit, cb);
                    continue;
                }

                DogLog.logFault("ExtendedLogger: @TunableField unsupported type " +
                        t.getSimpleName() + " for " + instance.getClass().getSimpleName() + "." + f.getName());

            } catch (Exception e) {
                DogLog.logFault("ExtendedLogger: tunable bind failed for " +
                        instance.getClass().getSimpleName() + "." + f.getName() + ": " + e.getMessage());
            }
        }
    }

    private static String resolveTunableKey(Object instance, Field f, String annotatedKey) {
        if (annotatedKey != null && !annotatedKey.isBlank())
            return annotatedKey;
        return "Tunable/" + instance.getClass().getSimpleName() + "/" + f.getName();
    }

    // ---- Tunable wrappers (avoid overload ambiguity + optional unit overload) ----

    private static void tunableDouble(String key, double def, String unit, DoubleConsumer onChange) {
        if (unit != null && !unit.isBlank()) {
            try {
                Method m = DogLog.class.getMethod("tunable",
                        String.class, double.class, String.class, DoubleConsumer.class);
                m.invoke(null, key, def, unit, onChange);
                return;
            } catch (Throwable ignored) {
            }
        }
        DogLog.tunable(key, def, (DoubleConsumer) onChange);
    }

    private static void tunableLong(String key, long def, String unit, LongConsumer onChange) {
        if (unit != null && !unit.isBlank()) {
            try {
                Method m = DogLog.class.getMethod("tunable",
                        String.class, long.class, String.class, LongConsumer.class);
                m.invoke(null, key, def, unit, onChange);
                return;
            } catch (Throwable ignored) {
            }
        }
        DogLog.tunable(key, def, (LongConsumer) onChange);
    }

    /** DogLog boolean tunable has no unit overload = unit ignored. */
    private static void tunableBoolean(String key, boolean def, String unit, BooleanConsumer onChange) {
        DogLog.tunable(key, def, (BooleanConsumer) onChange);
    }

    private static void tunableString(String key, String def, String unit, Consumer<String> onChange) {
        if (unit != null && !unit.isBlank()) {
            try {
                Method m = DogLog.class.getMethod("tunable",
                        String.class, String.class, String.class, Consumer.class);
                m.invoke(null, key, def, unit, onChange);
                return;
            } catch (Throwable ignored) {
            }
        }
        DogLog.tunable(key, def, (Consumer<String>) onChange);
    }

    // ---- Field setters ----

    private static double clamp(double v, double min, double max) {
        if (Double.isFinite(min) && v < min)
            v = min;
        if (Double.isFinite(max) && v > max)
            v = max;
        return v;
    }

    private static void setDouble(Object inst, Field f, double v) {
        try {
            if (f.getType() == double.class)
                f.setDouble(inst, v);
            else
                f.set(inst, v);
        } catch (Exception ignored) {
        }
    }

    private static void setFloat(Object inst, Field f, float v) {
        try {
            if (f.getType() == float.class)
                f.setFloat(inst, v);
            else
                f.set(inst, v);
        } catch (Exception ignored) {
        }
    }

    private static void setLong(Object inst, Field f, long v) {
        try {
            if (f.getType() == long.class)
                f.setLong(inst, v);
            else
                f.set(inst, v);
        } catch (Exception ignored) {
        }
    }

    private static void setInt(Object inst, Field f, int v) {
        try {
            if (f.getType() == int.class)
                f.setInt(inst, v);
            else
                f.set(inst, v);
        } catch (Exception ignored) {
        }
    }

    private static void setBoolean(Object inst, Field f, boolean v) {
        try {
            if (f.getType() == boolean.class)
                f.setBoolean(inst, v);
            else
                f.set(inst, v);
        } catch (Exception ignored) {
        }
    }

    private static void setObject(Object inst, Field f, Object v) {
        try {
            f.set(inst, v);
        } catch (Exception ignored) {
        }
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    private static void setEnumFromName(Object inst, Field f, String name) {
        try {
            Class<?> t = f.getType();
            if (!t.isEnum())
                return;
            if (name == null || name.isBlank())
                return;

            Class<? extends Enum> et = (Class<? extends Enum>) t;
            Enum val = Enum.valueOf(et, name);
            f.set(inst, val);
        } catch (Exception ignored) {
        }
    }

    // =========================
    // Fault throttling
    // =========================

    private static void logFaultThrottled(String key, String msg, int tick) {
        Integer last = faultCooldown.get(key);
        if (last == null || (tick - last) >= FAULT_COOLDOWN_TICKS) {
            faultCooldown.put(key, tick);
            DogLog.logFault(msg);
        }
    }

    // =========================
    // Records (cached explode)
    // =========================

    private static final class RecordPlan {
        final String[] names;
        final Method[] accessors;

        RecordPlan(String[] names, Method[] accessors) {
            this.names = names;
            this.accessors = accessors;
        }
    }

    private static void logRecord(String path, Record rec) {
        try {
            Class<?> cls = rec.getClass();
            RecordPlan plan = recordPlans.get(cls);
            if (plan == null) {
                RecordComponent[] comps = cls.getRecordComponents();
                String[] names = new String[comps.length];
                Method[] accessors = new Method[comps.length];

                for (int i = 0; i < comps.length; i++) {
                    names[i] = comps[i].getName();
                    accessors[i] = comps[i].getAccessor();
                    try {
                        accessors[i].setAccessible(true);
                    } catch (Exception ignored) {
                    }
                }

                plan = new RecordPlan(names, accessors);
                recordPlans.put(cls, plan);
            }

            for (int i = 0; i < plan.names.length; i++) {
                Object v = plan.accessors[i].invoke(rec);
                logGeneric(path + "/" + plan.names[i], v);
            }
        } catch (Exception e) {
            int tick = tickCounter.get();
            logFaultThrottled(
                    "record:" + path,
                    "Erreur record " + rec.getClass().getSimpleName() + " à " + path + ": " + e.getMessage(),
                    tick);
        }
    }

    // =========================
    // Generic fallback (rare path)
    // =========================

    private static void logGeneric(String path, Object value) {
        if (value == null)
            return;

        // 1) exact handler
        BiConsumer<String, Object> handler = handlers.get(value.getClass());
        if (handler != null) {
            handler.accept(path, value);
            return;
        }

        // 2) boxed arrays (convert)
        if (value instanceof Double[] a) {
            DogLog.log(path, toPrimitive(a));
            return;
        }
        if (value instanceof Boolean[] a) {
            DogLog.log(path, toPrimitive(a));
            return;
        }
        if (value instanceof Integer[] a) {
            DogLog.log(path, toPrimitive(a));
            return;
        }
        if (value instanceof Long[] a) {
            DogLog.log(path, toPrimitive(a));
            return;
        }
        if (value instanceof Float[] a) {
            DogLog.log(path, toPrimitive(a));
            return;
        }

        // 3) assignable last resort
        if (value instanceof Measure<?>) {
            DogLog.log(path, (Measure<?>) value);
            return;
        }
        if (value instanceof Record) {
            logRecord(path, (Record) value);
            return;
        }
        if (value instanceof Enum<?>) {
            DogLog.log(path, ((Enum<?>) value).name());
            return;
        }

        // 4) fallback string
        DogLog.log(path, String.valueOf(value));
    }

    private static double[] toPrimitive(Double[] a) {
        double[] out = new double[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : 0.0;
        return out;
    }

    private static boolean[] toPrimitive(Boolean[] a) {
        boolean[] out = new boolean[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : false;
        return out;
    }

    private static int[] toPrimitive(Integer[] a) {
        int[] out = new int[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : 0;
        return out;
    }

    private static long[] toPrimitive(Long[] a) {
        long[] out = new long[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : 0L;
        return out;
    }

    private static float[] toPrimitive(Float[] a) {
        float[] out = new float[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : 0f;
        return out;
    }

    // =========================
    // Identity key (no equals/hashCode surprises)
    // =========================

    private static final class IdKey {
        private final Object obj;
        private final int hash;

        private IdKey(Object obj) {
            this.obj = obj;
            this.hash = System.identityHashCode(obj);
        }

        static IdKey of(Object obj) {
            return new IdKey(obj);
        }

        @Override
        public int hashCode() {
            return hash;
        }

        @Override
        public boolean equals(Object other) {
            return (other instanceof IdKey k) && (k.obj == this.obj);
        }
    }

    // =========================
    // Annotations
    // =========================

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.FIELD)
    public @interface LoggableField {
        /** Chemin logger. Si vide = "ClassName/fieldName". */
        String path() default "";

        /** Fréquence cible (Hz). */
        double hz() default 50.0;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.FIELD)
    public @interface TunableField {
        /** Key tunable. Si vide = "Tunable/ClassName/fieldName" */
        String key() default "";

        /** Unité metadata (optionnel). */
        String unit() default "";

        /** Clamp min/max (si tu veux pas de clamp, mets NaN). */
        double min() default Double.NaN;

        double max() default Double.NaN;
    }

    private ExtendedLogger() {
    }
}