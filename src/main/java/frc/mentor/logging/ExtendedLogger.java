package frc.mentor.logging;

/**
 * (OLD loggingREADME.java file) API exposée aux jeunes (logging / AdvantageKit)
 *
 * - Log.value(String key, double v)
 * - Log.value(String key, int v)
 * - Log.value(String key, boolean v)
 * - Log.value(String key, String v)
 *
 * - Log.event(String key, String msg)
 * - Log.warn(String key, String msg)
 * - Log.error(String key, int code, String msg)
 * - Log.clearError(String key)
 *
 * - Log.time(String name, Runnable fn)
 * - {@code <T> T Log.time(String name, java.util.function.Supplier<T> fn)}
 * - AutoCloseable Log.scope(String name)
 *
 * - Log.everyN(int n, Runnable fn)
 * - Log.oncePerSecond(String key, Runnable fn)
 */

import java.lang.annotation.*;
import java.lang.reflect.Field;
import java.lang.reflect.RecordComponent;
import java.util.*;
import java.util.function.BiConsumer;

import dev.doglog.DogLog;
import edu.wpi.first.units.Measure;

/**
 * <p>
 * Logger simple pour les jeunes qui wrap DogLog.
 * </p>
 *
 * Objectif:
 * - Tu mets {@link LoggableField} sur des fields, tu register ton instance, pis ça log tout tout seul.
 * - Supporte les types de base, les tableaux et les Measures WPILib.
 * - Supporte les records en les "explosant" en sous-champs (path/nomDuChamp).
 *
 * Notes:
 * - Il faut appeler {@link #run()} dans robotPeriodic().
 * - Il faut appeler {@link #registerInstance(Object)} dans les constructors des
 * subsystems/objets à logger.
 */
public class ExtendedLogger extends DogLog {

    /** Instances qu'on a enregistrées (ex: subsystems) */
    private static final List<Object> registeredInstances = new ArrayList<>();

    /**
     * Set pour éviter d'enregistrer 2 fois le même objet.
     * IdentityHashMap = compare par référence (==), pas par equals().
     */
    private static final Set<Object> registeredSet = Collections.newSetFromMap(new IdentityHashMap<>());

    /**
     * Anti-spam pour éviter de spammer des erreurs à 50Hz.
     * key : dernier tick où on a loggé cette erreur.
     */
    private static final Map<String, Integer> faultCooldown = new HashMap<>();

    /** Compteur de loops (augmente à chaque run) */
    private static int loopCounter = 0;

    /** Cooldown en nombre de loops avant de relogger la même erreur. */
    private static final int FAULT_COOLDOWN_LOOPS = 50;

    /**
     * Log une erreur avec un throttle.
     *
     * @param key clé unique pour cette erreur (ex: "null:Shooter/rpm")
     * @param msg message à afficher
     */
    private static void logFaultThrottled(String key, String msg) {
        Integer last = faultCooldown.get(key);
        if (last == null || (loopCounter - last) >= FAULT_COOLDOWN_LOOPS) {
            faultCooldown.put(key, loopCounter);
            logFault(msg);
        }
    }

    /**
     * Map "type -> handler".
     * Handler = fonction qui sait appeler le bon DogLog.log() selon le type.
     */
    private static final Map<Class<?>, BiConsumer<String, Object>> handlers = new HashMap<>();

    static {
        handlers.put(Boolean.class, (p, o) -> log(p, (Boolean) o));
        handlers.put(Double.class, (p, o) -> log(p, (Double) o));
        handlers.put(Float.class, (p, o) -> log(p, (Float) o));
        handlers.put(Integer.class, (p, o) -> log(p, (Integer) o));
        handlers.put(Long.class, (p, o) -> log(p, (Long) o));
        handlers.put(String.class, (p, o) -> log(p, (String) o));
        handlers.put(Measure.class, (p, o) -> log(p, (Measure<?>) o));
        handlers.put(boolean[].class, (p, o) -> log(p, (boolean[]) o));
        handlers.put(double[].class, (p, o) -> log(p, (double[]) o));
        handlers.put(float[].class, (p, o) -> log(p, (float[]) o));
        handlers.put(int[].class, (p, o) -> log(p, (int[]) o));
        handlers.put(long[].class, (p, o) -> log(p, (long[]) o));
        handlers.put(String[].class, (p, o) -> log(p, (String[]) o));
        handlers.put(Record.class, (p, o) -> logRecord(p, (Record) o));
    }

    /**
     * Enregistre une instance (ex: subsystem) pour que ses fields annotés
     * {@link LoggableField} soient loggés automatiquement.
     *
     * @param instance objet à enregistrer
     */
    public static void registerInstance(Object instance) {
        if (instance == null)
            return;

        // même objet exact ne s'enregistre pas 2 fois
        if (registeredSet.add(instance)) {
            registeredInstances.add(instance);
        }
    }

    /**
     * À appeler une fois par loop dans robotPeriodic().
     * Ça scanne toutes les instances enregistrées et ça log les fields annotés.
     *
     */
    public static void run() {
        loopCounter++;
        logAll();
    }

    /**
     * Log tous les fields annotés {@link LoggableField} sur toutes les instances
     * enregistrées.
     *
     */
    public static void logAll() {
        for (Object inst : registeredInstances) {
            Field[] fields = inst.getClass().getDeclaredFields();

            for (Field field : fields) {
                if (!field.isAnnotationPresent(LoggableField.class))
                    continue;

                String path = field.getAnnotation(LoggableField.class).path();

                try {
                    // même si le field est private
                    field.setAccessible(true);

                    Object value = field.get(inst);
                    logGeneric(path, value);

                } catch (IllegalAccessException e) {
                    logFaultThrottled(
                            "illegalAccess:" + inst.getClass().getName() + "." + field.getName(),
                            "Accès refusé au field " + field.getName() + " (private / security Java?)");
                } catch (IllegalArgumentException e) {
                    logFaultThrottled(
                            "illegalArg:" + inst.getClass().getName() + "." + field.getName(),
                            "IllegalArgument sur " + field.getName() + ": " + e.getMessage());
                } catch (Exception e) {
                    logFaultThrottled(
                            "other:" + inst.getClass().getName() + "." + field.getName(),
                            "Erreur logging " + field.getName() + ": "
                                    + e.getClass().getSimpleName() + " " + e.getMessage());
                }
            }
        }
    }

    /**
     * Log une valeur quand on ne sait pas d'avance c'est quel type.
     *
     * Règles:
     * - On essaie d'abord un handler exact (value.getClass()).
     * - Ensuite on gère les tableaux boxed (Double[]) en les convertissant en primitifs (double[]).
     * - Ensuite on essaie les types assignables (ex: Record, Measure).
     * - Sinon on fallback sur value.toString().
     *
     * @param path  chemin DogLog (ex: "Shooter/rpm")
     * @param value valeur à logger
     */
    private static void logGeneric(String path, Object value) {
        if (value == null) {
            logFaultThrottled("null:" + path, "Impossible de logger une valeur null à " + path);
            return;
        }

        // 1 - Handler exact
        BiConsumer<String, Object> handler = handlers.get(value.getClass());
        if (handler != null) {
            handler.accept(path, value);
            return;
        }

        // 2 - Tableaux boxed convert en primitifs
        if (value instanceof Double[] a) {
            handlers.get(double[].class).accept(path, toPrimitive(a));
            return;
        }
        if (value instanceof Boolean[] a) {
            handlers.get(boolean[].class).accept(path, toPrimitive(a));
            return;
        }
        if (value instanceof Integer[] a) {
            handlers.get(int[].class).accept(path, toPrimitive(a));
            return;
        }
        if (value instanceof Long[] a) {
            handlers.get(long[].class).accept(path, toPrimitive(a));
            return;
        }
        if (value instanceof Float[] a) {
            handlers.get(float[].class).accept(path, toPrimitive(a));
            return;
        }

        // 3 - Handler assignable (ex: Record, Measure)
        for (Map.Entry<Class<?>, BiConsumer<String, Object>> entry : handlers.entrySet()) {
            if (entry.getKey().isAssignableFrom(value.getClass())) {
                entry.getValue().accept(path, value);
                return;
            }
        }

        // 4 - Fallback: log string (C'est mieux qu'une erreur)
        log(path, value.toString());
    }

    /**
     * Log un Record en le séparant en sous-champs.
     *
     * Exemple:
     * - path = "ShooterState"
     * - record = ShooterState(rpm=3000, hoodDeg=60, ...)
     * Ça donne :
     * ShooterState/rpm
     * ShooterState/hoodDeg
     *
     * @param path chemin racine
     * @param rec  record à logger
     */
    private static void logRecord(String path, Record rec) {
        try {
            RecordComponent[] comps = rec.getClass().getRecordComponents();
            for (RecordComponent c : comps) {
                Object v = c.getAccessor().invoke(rec);
                logGeneric(path + "/" + c.getName(), v);
            }
        } catch (Exception e) {
            logFaultThrottled(
                    "record:" + path,
                    "Erreur record " + rec.getClass().getSimpleName() + " à " + path + ": " + e.getMessage());
        }
    }

    /**
     * Convertit Double[] en double[] (null devient 0.0).
     *
     * @param a tableau boxed
     */
    private static double[] toPrimitive(Double[] a) {
        double[] out = new double[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : 0.0;
        return out;
    }

    /**
     * Convertit Boolean[] en boolean[] (null devient false).
     *
     * @param a tableau boxed
     */
    private static boolean[] toPrimitive(Boolean[] a) {
        boolean[] out = new boolean[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : false;
        return out;
    }

    /**
     * Convertit Integer[] en int[] (null devient 0).
     *
     * @param a tableau boxed
     */
    private static int[] toPrimitive(Integer[] a) {
        int[] out = new int[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : 0;
        return out;
    }

    /**
     * Convertit Long[] en long[] (null devient 0L).
     *
     * @param a tableau boxed
     */
    private static long[] toPrimitive(Long[] a) {
        long[] out = new long[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : 0L;
        return out;
    }

    /**
     * Convertit Float[] en float[] (null devient 0f).
     *
     * @param a tableau boxed
     */
    private static float[] toPrimitive(Float[] a) {
        float[] out = new float[a.length];
        for (int i = 0; i < a.length; i++)
            out[i] = (a[i] != null) ? a[i] : 0f;
        return out;
    }

    /**
     * Annotation à mettre sur un field pour qu'il soit loggé.
     *
     * Exemple:
     * {@code
     * @ExtendedLogger.LoggableField(path="Shooter/solver_ms")
     * private double solverMs;
     * }
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.FIELD)
    public @interface LoggableField {

        /**
         * Chemin sous lequel logger la valeur.
         */
        String path();
    }
}
