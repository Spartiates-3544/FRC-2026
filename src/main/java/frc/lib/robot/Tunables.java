package frc.lib.robot;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * <p>
 * Tunables
 * </p>
 *
 * <p>
 * Petite couche au-dessus de {@link Config} (NetworkTables /Tuning) :
 * </p>
 * <ul>
 * <li>TunableNumber / TunableBoolean / TunableString (objet)</li>
 * <li>Update rate optionnel (ne pas lire NT à 50Hz si tu veux pas)</li>
 * <li>Freeze global (lock pendant un match)</li>
 * <li>Callback de logging optionnel (hook ton ExtendedLogger)</li>
 * </ul>
 *
 * <p>
 * BONUS: support par annotations (comme ton ExtendedLogger) :
 * </p>
 * <ul>
 * <li>Tu mets @TunableNum / @TunableBool / @TunableStr sur des fields</li>
 * <li>Tu appelles {@code Tunables.registerInstance(this)} dans le constructeur</li>
 * <li>Tu appelles {@code Tunables.run()} une fois par loop (robotPeriodic)</li>
 * <li>Les fields se mettent à jour automatiquement depuis NT (et publient les defaults)</li>
 * </ul>
 *
 * <p>
 * Les clés vont dans :
 * </p>
 * 
 * <pre>{@code
 * /Tuning/<key>
 * Exemple: /Tuning/Motion/Climb/Pivot/maxVelRadS
 * }</pre>
 */
public final class Tunables {
    private Tunables() {
    }

    // =========================================================
    // Contrôles globaux
    // =========================================================

    /** Freeze global: si true, on ne met plus à jour depuis NT (valeur = last-known). */
    private static volatile boolean FROZEN = false;

    /** Source de temps (pour tests unitaires, tu peux remplacer). */
    private static Supplier<Double> timeNow = Timer::getFPGATimestamp;

    /** Freeze tous les tunables (ils gardent leur dernière valeur). */
    public static void freeze() {
        FROZEN = true;
    }

    /** Unfreeze (les tunables recommencent à lire NT). */
    public static void unfreeze() {
        FROZEN = false;
    }

    public static boolean isFrozen() {
        return FROZEN;
    }

    /** Pour tests: remplace la source de temps. */
    public static void setTimeSource(Supplier<Double> timeSource) {
        timeNow = timeSource;
    }

    // =========================================================
    // Registry pour les tunables "objet" (TunableNumber, etc.)
    // =========================================================

    /**
     * Registry global (key -> BaseTunable).
     * Sert à:
     * - réutiliser le même tunable si le key est demandé 2 fois
     * - détecter si quelqu’un réutilise le même key avec un type différent
     */
    private static final Map<String, BaseTunable> REGISTRY = new ConcurrentHashMap<>();

    // -----------------------------
    // Factories (type-safe)
    // -----------------------------

    public static TunableNumber number(String key, double defaultValue) {
        return typed(key, TunableNumber.class, () -> new TunableNumber(key, defaultValue));
    }

    public static TunableBoolean bool(String key, boolean defaultValue) {
        return typed(key, TunableBoolean.class, () -> new TunableBoolean(key, defaultValue));
    }

    public static TunableString string(String key, String defaultValue) {
        return typed(key, TunableString.class, () -> new TunableString(key, defaultValue));
    }

    /**
     * Helper interne:
     * - si le key existe déjà, on le retourne
     * - sinon on le crée
     * - si le key existe mais pas le bon type => erreur claire (ça évite des bugs weird)
     */
    private static <T extends BaseTunable> T typed(String key, Class<T> type, Supplier<T> creator) {
        BaseTunable existing = REGISTRY.get(key);
        if (existing != null) {
            if (!type.isInstance(existing)) {
                throw new IllegalStateException(
                        "Tunables key reused with different type: key=\"" + key + "\" existing="
                                + existing.getClass().getSimpleName() + " new=" + type.getSimpleName());
            }
            return type.cast(existing);
        }

        T created = creator.get();
        BaseTunable prev = REGISTRY.putIfAbsent(key, created);
        return (prev == null) ? created : type.cast(prev);
    }

    /** Optionnel: dump toutes les valeurs actuelles (pratique au enable). */
    public static void logSnapshotToStdout(String title) {
        System.out.println("---- Tunables Snapshot: " + title + " ----");
        REGISTRY.forEach((k, v) -> System.out.println("/Tuning/" + k + " = " + v.valueAsString()));
        System.out.println("------------------------------------------");
    }

    // =========================================================
    // Tunables par annotations (comme ExtendedLogger)
    // =========================================================

    /**
     * <p>
     * Annoter un field double/Double pour le rendre "tunable" via /Tuning/<key>.
     * </p>
     *
     * Exemple:
     * 
     * <pre>{@code
     * @TunableNum(key = "Drive/maxSpeed", def = 4.5, hz = 2, clamp = true, min = 0, max = 6)
     * private double maxSpeed;
     * }</pre>
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.FIELD)
    public @interface TunableNum {
        /** Key sous /Tuning (ex: "Drive/maxSpeed") */
        String key();

        /** Default publié (setDefaultDouble) si rien n’existe encore dans NT. */
        double def();

        /** Fréquence de refresh (Hz). */
        double hz() default 5.0;

        /** Clamp de sécurité optionnel. */
        boolean clamp() default false;

        /** Min clamp (si clamp=true). */
        double min() default -1e9;

        /** Max clamp (si clamp=true). */
        double max() default +1e9;
    }

    /**
     * Annoter un field boolean/Boolean pour le rendre tunable.
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.FIELD)
    public @interface TunableBool {
        String key();

        boolean def();

        double hz() default 5.0;
    }

    /**
     * Annoter un field String pour le rendre tunable.
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.FIELD)
    public @interface TunableStr {
        String key();

        String def();

        double hz() default 2.0;
    }

    /** Liste des instances qu’on scanne (subsystems, objets, etc.) */
    private static final List<Object> ANNOTATED_INSTANCES = new ArrayList<>();

    /**
     * Set identité pour ne pas enregistrer le même objet 2 fois.
     * IdentityHashMap = compare par référence (==), pas equals().
     */
    private static final Set<Object> ANNOTATED_SET = Collections.newSetFromMap(new IdentityHashMap<>());

    /**
     * <p>
     * Enregistre un objet qui contient des @TunableX fields.
     * </p>
     *
     * <p>
     * À appeler dans le constructeur du subsystem :
     * 
     * <pre>{@code
     * Tunables.registerInstance(this);
     * }</pre>
     * </p>
     */
    public static void registerInstance(Object instance) {
        if (instance == null)
            return;
        if (ANNOTATED_SET.add(instance)) {
            ANNOTATED_INSTANCES.add(instance);
        }
    }

    /**
     * <p>
     * À appeler une fois par loop dans robotPeriodic().
     * </p>
     *
     * <p>
     * Ce que ça fait:
     * </p>
     * <ul>
     * <li>Pour chaque instance enregistrée, scan ses fields</li>
     * <li>Si @TunableNum/@TunableBool/@TunableStr: lit la valeur depuis NT</li>
     * <li>Écrit la valeur directement dans le field</li>
     * </ul>
     *
     * <p>
     * Important:
     * </p>
     * <ul>
     * <li>On n’écrase pas ce qui existe déjà dans NT: {@link Config} publie les defaults via setDefaultXxx</li>
     * <li>Si frozen: rien ne bouge</li>
     * <li>On catch les erreurs pour ne JAMAIS faire crasher le robot</li>
     * </ul>
     */
    public static void run() {
        if (FROZEN)
            return;

        for (Object inst : ANNOTATED_INSTANCES) {
            Field[] fields = inst.getClass().getDeclaredFields();

            for (Field f : fields) {
                try {
                    // -------------------------
                    // Number (double/Double)
                    // -------------------------
                    if (f.isAnnotationPresent(TunableNum.class)) {
                        var a = f.getAnnotation(TunableNum.class);
                        f.setAccessible(true);

                        // On crée/récupère le tunable (key unique) et on applique le rate
                        TunableNumber t = Tunables.number(a.key(), a.def()).withUpdateHz(a.hz());

                        // Clamp de sécurité optionnel (ça protège des valeurs folles sur le dashboard)
                        if (a.clamp())
                            t.withClamp(a.min(), a.max());

                        // Update (respecte le rate-limit interne)
                        t.update();

                        // Écrit la valeur dans le field
                        Class<?> ft = f.getType();
                        if (ft == double.class) {
                            f.setDouble(inst, t.get());
                        } else if (ft == Double.class) {
                            f.set(inst, t.get());
                        } else {
                            throw new IllegalStateException("@" + TunableNum.class.getSimpleName()
                                    + " field must be double/Double: " + inst.getClass().getSimpleName()
                                    + "." + f.getName());
                        }
                    }

                    // -------------------------
                    // Boolean (boolean/Boolean)
                    // -------------------------
                    if (f.isAnnotationPresent(TunableBool.class)) {
                        var a = f.getAnnotation(TunableBool.class);
                        f.setAccessible(true);

                        TunableBoolean t = Tunables.bool(a.key(), a.def()).withUpdateHz(a.hz());
                        t.update();

                        Class<?> ft = f.getType();
                        if (ft == boolean.class) {
                            f.setBoolean(inst, t.get());
                        } else if (ft == Boolean.class) {
                            f.set(inst, t.get());
                        } else {
                            throw new IllegalStateException("@" + TunableBool.class.getSimpleName()
                                    + " field must be boolean/Boolean: " + inst.getClass().getSimpleName()
                                    + "." + f.getName());
                        }
                    }

                    // -------------------------
                    // String
                    // -------------------------
                    if (f.isAnnotationPresent(TunableStr.class)) {
                        var a = f.getAnnotation(TunableStr.class);
                        f.setAccessible(true);

                        TunableString t = Tunables.string(a.key(), a.def()).withUpdateHz(a.hz());
                        t.update();

                        if (f.getType() == String.class) {
                            f.set(inst, t.get());
                        } else {
                            throw new IllegalStateException("@" + TunableStr.class.getSimpleName()
                                    + " field must be String: " + inst.getClass().getSimpleName()
                                    + "." + f.getName());
                        }
                    }

                } catch (Exception e) {
                    // On ne veut JAMAIS planter le robot juste parce qu’un tunable est mal configuré.
                    System.out.println("[Tunables] Échec update "
                            + inst.getClass().getSimpleName() + "." + f.getName()
                            + " : " + e.getClass().getSimpleName() + " " + e.getMessage());
                }
            }
        }
    }

    // =========================================================
    // Classe de base (commun à tous les types)
    // =========================================================

    private abstract static class BaseTunable {
        protected final String key;

        /** Période d’update (sec). Default = 5 Hz. */
        protected double updatePeriodSec = 1.0 / 5.0;

        /** Dernier moment où on a réellement tiré de NT. */
        protected double lastUpdateTime = -1.0;

        BaseTunable(String key) {
            this.key = key;
        }

        /**
         * Update:
         * - respecte le freeze
         * - respecte le rate-limit (updatePeriodSec)
         * - lit NT via pullFromNt()
         * - puis appelle onValueUpdated() (log hook optionnel)
         */
        public final void update() {
            if (FROZEN)
                return;

            double now = timeNow.get();
            if (lastUpdateTime < 0) {
                // Première fois: on lit tout de suite
                lastUpdateTime = now;
                pullFromNt();
                onValueUpdated();
                return;
            }

            // Ensuite: seulement si la période est écoulée
            if ((now - lastUpdateTime) >= updatePeriodSec) {
                lastUpdateTime = now;
                pullFromNt();
                onValueUpdated();
            }
        }

        /** Lit NT et met à jour la valeur en mémoire. */
        protected abstract void pullFromNt();

        /** Hook appelé après un update (ex: log). */
        protected abstract void onValueUpdated();

        /** Pour snapshot debug. */
        protected abstract String valueAsString();
    }

    // =========================================================
    // TunableNumber
    // =========================================================

    public static final class TunableNumber extends BaseTunable {
        private final double defaultValue;
        private volatile double value;

        // Clamp optionnel
        private boolean enableClamp = false;
        private double clampMin = 0.0;
        private double clampMax = 0.0;

        // Hook log optionnel
        private DoubleConsumer logFn = null;

        private TunableNumber(String key, double defaultValue) {
            super(key);
            this.defaultValue = defaultValue;

            // Publie le default (setDefaultDouble) sans écraser si le DS a déjà une valeur.
            value = Config.getNumber(key, defaultValue);
        }

        /** Change la fréquence de refresh (Hz). */
        public TunableNumber withUpdateHz(double hz) {
            double safeHz = Math.max(0.1, hz);
            updatePeriodSec = 1.0 / safeHz;
            return this;
        }

        /** Active un clamp min/max (sécurité). */
        public TunableNumber withClamp(double min, double max) {
            enableClamp = true;
            clampMin = min;
            clampMax = max;
            value = MathUtil.clamp(value, clampMin, clampMax);
            return this;
        }

        /** Callback après chaque update effectif. */
        public TunableNumber withLogging(DoubleConsumer loggerFn) {
            this.logFn = loggerFn;
            return this;
        }

        public double get() {
            return value;
        }

        public double getDefault() {
            return defaultValue;
        }

        /**
         * Force une valeur (écrit dans NT).
         * Utile pour un bouton "Apply autotune".
         */
        public void set(double newValue) {
            if (enableClamp)
                newValue = MathUtil.clamp(newValue, clampMin, clampMax);
            value = newValue;
            Config.setNumber(key, newValue);
            onValueUpdated();
        }

        @Override
        protected void pullFromNt() {
            double v = Config.getNumber(key, defaultValue);
            if (enableClamp)
                v = MathUtil.clamp(v, clampMin, clampMax);
            value = v;
        }

        @Override
        protected void onValueUpdated() {
            if (logFn != null)
                logFn.accept(value);
        }

        @Override
        protected String valueAsString() {
            return Double.toString(value);
        }
    }

    // =========================================================
    // TunableBoolean
    // =========================================================

    public static final class TunableBoolean extends BaseTunable {
        private final boolean defaultValue;
        private volatile boolean value;

        private Consumer<Boolean> logFn = null;

        private TunableBoolean(String key, boolean defaultValue) {
            super(key);
            this.defaultValue = defaultValue;
            value = Config.getBoolean(key, defaultValue);
        }

        /** Change la fréquence de refresh (Hz). */
        public TunableBoolean withUpdateHz(double hz) {
            double safeHz = Math.max(0.1, hz);
            updatePeriodSec = 1.0 / safeHz;
            return this;
        }

        public TunableBoolean withLogging(Consumer<Boolean> loggerFn) {
            this.logFn = loggerFn;
            return this;
        }

        public boolean get() {
            return value;
        }

        public boolean getDefault() {
            return defaultValue;
        }

        /** Force une valeur (écrit dans NT). */
        public void set(boolean newValue) {
            value = newValue;
            Config.setBoolean(key, newValue);
            onValueUpdated();
        }

        @Override
        protected void pullFromNt() {
            value = Config.getBoolean(key, defaultValue);
        }

        @Override
        protected void onValueUpdated() {
            if (logFn != null)
                logFn.accept(value);
        }

        @Override
        protected String valueAsString() {
            return Boolean.toString(value);
        }
    }

    // =========================================================
    // TunableString
    // =========================================================

    public static final class TunableString extends BaseTunable {
        private final String defaultValue;
        private volatile String value;

        private Consumer<String> logFn = null;

        private TunableString(String key, String defaultValue) {
            super(key);
            this.defaultValue = defaultValue;
            value = Config.getString(key, defaultValue);
        }

        /** Change la fréquence de refresh (Hz). */
        public TunableString withUpdateHz(double hz) {
            double safeHz = Math.max(0.1, hz);
            updatePeriodSec = 1.0 / safeHz;
            return this;
        }

        public TunableString withLogging(Consumer<String> loggerFn) {
            this.logFn = loggerFn;
            return this;
        }

        public String get() {
            return value;
        }

        public String getDefault() {
            return defaultValue;
        }

        /** Force une valeur (écrit dans NT). */
        public void set(String newValue) {
            value = newValue;
            Config.setString(key, newValue);
            onValueUpdated();
        }

        @Override
        protected void pullFromNt() {
            value = Config.getString(key, defaultValue);
        }

        @Override
        protected void onValueUpdated() {
            if (logFn != null)
                logFn.accept(value);
        }

        @Override
        protected String valueAsString() {
            return value;
        }
    }
}
