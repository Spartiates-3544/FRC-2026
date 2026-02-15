// frc/mentor/robot/Config.java
package frc.mentor.robot;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * <p>
 * Config/Tunables pour les NetworkTables.
 * </p>
 *
 * <h2>But</h2>
 * <ul>
 * <li>Tuner des valeurs en temps réel</li>
 * <li>Unifier les clés</li>
 * <li>Publier des "defaults" sans écraser ce qui est déjà sur le DS</li>
 * </ul>
 *
 * <h2>Chemin NT</h2>
 * 
 * <pre>{@code
 * /Tuning/<key>
 * Ex: /Tuning/Shooter/slipFactor
 * Ex: /Tuning/Drive/maxSpeed
 * }</pre>
 *
 * <h2>Usage (recommandé)</h2>
 * 
 * <pre>{@code
 * double slip = Config.getNumber("Shooter/slipFactor", 0.90);
 * boolean drag = Config.getBoolean("Shooter/enableDrag", true);
 *
 * // Optionnel: log de toutes les valeurs actives (au enable)
 * Config.logActiveConfig();
 * }</pre>
 *
 * <p>
 * Notes:
 * <ul>
 * <li>{@link #getNumber(String, double)} et {@link #getBoolean(String, boolean)} publient un default
 * via {@code setDefaultXxx} (n'écrase jamais si une valeur existe déjà).</li>
 * <li>On garde un petit registry de toutes les clés lues pour pouvoir les lister/log.</li>
 * </ul>
 */
public final class Config {
    private Config() {
    }

    /** Table racine pour tous les tunables. */
    private static final NetworkTable ROOT = NetworkTableInstance.getDefault().getTable("Tuning");

    /** Registry des clés qu'on a déjà touchées */
    private static final Map<String, EntryInfo> REGISTRY = new ConcurrentHashMap<>();

    private enum Kind {
        NUMBER, BOOLEAN, STRING
    }

    private static final class EntryInfo {
        final Kind kind;
        final NetworkTableEntry entry;
        volatile Object defaultValue;

        EntryInfo(Kind kind, NetworkTableEntry entry, Object defaultValue) {
            this.kind = kind;
            this.entry = entry;
            this.defaultValue = defaultValue;
        }
    }

    /**
     * Lit un double tunable. Publie le default si la clé n'existe pas encore.
     *
     * @param key
     *            chemin relatif sous /Tuning (ex: "Shooter/slipFactor")
     * @param defaultValue
     *            valeur safe si rien n'est publié
     * @return valeur actuelle (NT) ou defaultValue
     */
    public static double getNumber(String key, double defaultValue) {
        NetworkTableEntry e = entry(key);
        e.setDefaultDouble(defaultValue);
        REGISTRY.putIfAbsent(key, new EntryInfo(Kind.NUMBER, e, defaultValue));
        return e.getDouble(defaultValue);
    }

    /**
     * Lit un boolean tunable. Publie le default si la clé n'existe pas encore.
     *
     * @param key
     *            chemin relatif sous /Tuning (ex: "Shooter/enableDrag")
     * @param defaultValue
     *            valeur safe si rien n'est publié
     * @return valeur actuelle (NT) ou defaultValue
     */
    public static boolean getBoolean(String key, boolean defaultValue) {
        NetworkTableEntry e = entry(key);
        e.setDefaultBoolean(defaultValue);
        REGISTRY.putIfAbsent(key, new EntryInfo(Kind.BOOLEAN, e, defaultValue));
        return e.getBoolean(defaultValue);
    }

    /**
     * Lit un string tunable. Publie le default si la clé n'existe pas encore.
     *
     * @param key
     *            chemin relatif sous /Tuning (ex: "Shooter/goalType")
     * @param defaultValue
     *            valeur safe si rien n'est publié
     * @return valeur actuelle (NT) ou defaultValue
     */
    public static String getString(String key, String defaultValue) {
        NetworkTableEntry e = entry(key);
        e.setDefaultString(defaultValue);
        REGISTRY.putIfAbsent(key, new EntryInfo(Kind.STRING, e, defaultValue));
        return e.getString(defaultValue);
    }

    /**
     * Force une valeur double dans NT.
     * Attention: ça écrase la valeur actuelle.
     */
    public static void setNumber(String key, double value) {
        NetworkTableEntry e = entry(key);
        e.setDouble(value);
        REGISTRY.putIfAbsent(key, new EntryInfo(Kind.NUMBER, e, value));
    }

    /**
     * Force une valeur boolean dans NT.
     * Attention: ça écrase la valeur actuelle.
     */
    public static void setBoolean(String key, boolean value) {
        NetworkTableEntry e = entry(key);
        e.setBoolean(value);
        REGISTRY.putIfAbsent(key, new EntryInfo(Kind.BOOLEAN, e, value));
    }

    /**
     * Force une valeur string dans NT.
     * Attention: ça écrase la valeur actuelle.
     */
    public static void setString(String key, String value) {
        NetworkTableEntry e = entry(key);
        e.setString(value);
        REGISTRY.putIfAbsent(key, new EntryInfo(Kind.STRING, e, value));
    }

    /**
     * Repousse les defaults pour toutes les clés déjà vues.
     */
    public static void publishAll() {
        for (var it : REGISTRY.entrySet()) {
            EntryInfo info = it.getValue();
            switch (info.kind) {
                case NUMBER -> info.entry.setDefaultDouble((double) info.defaultValue);
                case BOOLEAN -> info.entry.setDefaultBoolean((boolean) info.defaultValue);
                case STRING -> info.entry.setDefaultString((String) info.defaultValue);
            }
        }
    }

    /**
     * Log simple de toutes les keys vues (stdout).
     */
    public static void logActiveConfig() {
        System.out.println("---- Active Config (/Tuning) ----");
        REGISTRY.forEach((key, info) -> {
            String full = "/Tuning/" + key;
            switch (info.kind) {
                case NUMBER -> System.out.println(full + " = " + info.entry.getDouble((double) info.defaultValue));
                case BOOLEAN -> System.out.println(full + " = " + info.entry.getBoolean((boolean) info.defaultValue));
                case STRING -> System.out.println(full + " = " + info.entry.getString((String) info.defaultValue));
            }
        });
        System.out.println("---------------------------------");
    }

    private static NetworkTableEntry entry(String key) {
        // permet des keys avec /
        String[] parts = key.split("/");
        NetworkTable t = ROOT;
        for (int i = 0; i < parts.length - 1; i++) {
            if (!parts[i].isEmpty())
                t = t.getSubTable(parts[i]);
        }
        return t.getEntry(parts[parts.length - 1]);
    }
}
