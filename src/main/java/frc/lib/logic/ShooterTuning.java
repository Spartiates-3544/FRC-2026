package frc.lib.logic;

import java.lang.reflect.Constructor;
import java.lang.reflect.RecordComponent;
import java.util.Locale;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.lib.robot.Records;
import frc.robot.Constants;

/**
 * ShooterTuning:
 * - Publishes all Records.ShooterParams fields to NetworkTables (grouped)
 * - Loads Constants defaults ONCE per NT reset (latched)
 * - Reads tuned values back and reconstructs the record
 *
 * NT layout:
 * /Tuning/Shooter/_defaults_loaded (bool)
 * /Tuning/Shooter/Group/recordField
 */
public final class ShooterTuning {

    private static final String ROOT_PATH = "Tuning/Shooter";
    private static ShooterTuning instance;

    private final NetworkTable root;
    private final NetworkTableEntry defaultsLoaded;

    private final Records.ShooterParams defaults;

    private final RecordComponent[] comps;
    private final Constructor<?> canonicalCtor;

    public static ShooterTuning get() {
        if (instance == null)
            instance = new ShooterTuning();
        return instance;
    }

    private ShooterTuning() {
        this.root = NetworkTableInstance.getDefault().getTable(ROOT_PATH);
        this.defaultsLoaded = root.getEntry("_defaults_loaded");

        this.defaults = Constants.Shooter.defaultParams();

        this.comps = Records.ShooterParams.class.getRecordComponents();
        try {
            Class<?>[] sig = new Class<?>[comps.length];
            for (int i = 0; i < comps.length; i++)
                sig[i] = comps[i].getType();
            this.canonicalCtor = Records.ShooterParams.class.getDeclaredConstructor(sig);
            this.canonicalCtor.setAccessible(true);
        } catch (Exception e) {
            throw new RuntimeException("ShooterTuning: can't access ShooterParams canonical constructor", e);
        }

        loadDefaultsOnce();
    }

    /** Call this from robotInit to create early. */
    public static void warmup() {
        get();
    }

    /** Backward-compatible helper so old call-sites don't explode. */
    public static Records.ShooterParams params(Records.ShooterParams ignoredDefaults) {
        return get().params();
    }

    /** Publish defaults to NT exactly once per NT lifetime. */
    public void loadDefaultsOnce() {
        if (defaultsLoaded.getBoolean(false))
            return;

        for (RecordComponent rc : comps) {
            Object defVal = readDefaultValue(rc);
            NetworkTableEntry e = entryFor(rc.getName(), rc.getType());
            writeEntry(e, rc.getType(), defVal);
        }

        defaultsLoaded.setBoolean(true);
    }

    /** Current tuned params (from NT overrides, fallback to Constants defaults). */
    public Records.ShooterParams params() {
        if (!defaultsLoaded.getBoolean(false)) {
            loadDefaultsOnce();
        }

        Object[] args = new Object[comps.length];

        for (int i = 0; i < comps.length; i++) {
            RecordComponent rc = comps[i];
            Class<?> t = rc.getType();

            Object defVal = readDefaultValue(rc);
            NetworkTableEntry e = entryFor(rc.getName(), t);

            args[i] = readEntry(e, t, defVal);
        }

        try {
            return (Records.ShooterParams) canonicalCtor.newInstance(args);
        } catch (Exception e) {
            throw new RuntimeException("ShooterTuning: failed to construct ShooterParams from NT values", e);
        }
    }

    // =========================
    // Grouping
    // =========================

    private NetworkTableEntry entryFor(String fieldName, Class<?> type) {
        String group = groupFor(fieldName);
        NetworkTable table = root.getSubTable(group);
        return table.getEntry(fieldName);
    }

    private static String groupFor(String fieldName) {
        String n = fieldName.toLowerCase(Locale.ROOT);

        if (n.contains("blind") || n.contains("blindspot") || n.contains("occlusion"))
            return "BlindSpot";
        if (n.contains("iter") || n.contains("epsilon") || n.equals("eps") || n.contains("tolerance")
                || n.contains("solver") || n.contains("step") || n.contains("dt"))
            return "Solver";
        if (n.contains("min") || n.contains("max") || n.contains("limit") || n.contains("clamp")
                || n.contains("bound"))
            return "Limits";

        return "Params";
    }

    // =========================
    // Read/write
    // =========================

    private Object readDefaultValue(RecordComponent rc) {
        try {
            return rc.getAccessor().invoke(defaults);
        } catch (Exception e) {
            throw new RuntimeException("ShooterTuning: can't read default for " + rc.getName(), e);
        }
    }

    private static void writeEntry(NetworkTableEntry e, Class<?> t, Object v) {
        if (t == double.class || t == Double.class) {
            e.setDouble(((Number) v).doubleValue());
        } else if (t == int.class || t == Integer.class) {
            e.setInteger(((Number) v).longValue());
        } else if (t == boolean.class || t == Boolean.class) {
            e.setBoolean((Boolean) v);
        } else if (t == String.class) {
            e.setString((String) v);
        } else {
            throw new IllegalArgumentException("ShooterTuning: unsupported field type: " + t.getName());
        }
    }

    private static Object readEntry(NetworkTableEntry e, Class<?> t, Object defVal) {
        if (t == double.class || t == Double.class) {
            return e.getDouble(((Number) defVal).doubleValue());
        } else if (t == int.class || t == Integer.class) {
            long d = ((Number) defVal).longValue();
            return (int) e.getInteger(d);
        } else if (t == boolean.class || t == Boolean.class) {
            return e.getBoolean((Boolean) defVal);
        } else if (t == String.class) {
            return e.getString((String) defVal);
        } else {
            throw new IllegalArgumentException("ShooterTuning: unsupported field type: " + t.getName());
        }
    }
}