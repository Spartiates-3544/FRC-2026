package frc.mentor.utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;

/**
 * Utilitaires de filtrage pour capteurs et signaux de contrôle.
 *
 * Objectif:
 * - Fournir des wrappers simples et réutilisables pour les subsystems/commands.
 * - Utiliser les filtres WPILib quand c'est utile, sans compliquer l'usage.
 *
 * Règles:
 * - Aucun hardware ici (pas de moteurs, pas de capteurs). Juste du traitement
 * de signal.
 * - Les classes avec état (MovingAverage, RateLimiter, etc.) doivent être
 * stockées en champ
 * dans un subsystem pour conserver leur mémoire entre les cycles.
 * - Les helpers stateless (lowPass, median3, rejectOutliers) ne conservent
 * aucun état.
 */
public final class Filters {
    private Filters() {
    }

    // -------------------------
    // Stateless helpers
    // -------------------------

    /**
     * Filtre passe-bas (1er ordre) sans état.
     *
     * Formule: {@code y = prev + alpha * (x - prev)}.
     *
     * alpha contrôle la réactivité:
     * - alpha proche de 1.0: suit vite (peu de filtrage)
     * - alpha proche de 0.0: très smooth (fort filtrage)
     *
     * @param prev  valeur filtrée précédente (y[n-1])
     * @param x     nouvelle mesure brute (x[n])
     * @param alpha coefficient entre 0 et 1 (clampé automatiquement)
     * @return nouvelle valeur filtrée
     */
    public static double lowPass(double prev, double x, double alpha) {
        alpha = clamp01(alpha);
        return prev + alpha * (x - prev);
    }

    /**
     * Médiane de 3 valeurs (stateless).
     *
     * Utile pour enlever un spike ponctuel, sans garder d'historique.
     *
     * @param a valeur 1
     * @param b valeur 2
     * @param c valeur 3
     * @return la valeur du milieu (médiane)
     */
    public static double median3(double a, double b, double c) {
        if (a > b) {
            double t = a;
            a = b;
            b = t;
        }
        if (b > c) {
            double t = b;
            b = c;
            c = t;
        }
        if (a > b) {
            double t = a;
            a = b;
            b = t;
        }
        return b;
    }

    /**
     * Rejet simple d'outliers.
     *
     * Si {@code x} sort de l'intervalle {@code [lo, hi]}, retourne {@code prev}.
     *
     * @param prev dernière valeur acceptée
     * @param x    nouvelle valeur candidate
     * @param lo   borne minimale acceptable
     * @param hi   borne maximale acceptable
     * @return {@code x} si valide, sinon {@code prev}
     */
    public static double rejectOutliers(double prev, double x, double lo, double hi) {
        return (x < lo || x > hi) ? prev : x;
    }

    private static double clamp01(double x) {
        return Math.max(0.0, Math.min(1.0, x));
    }

    // -------------------------
    // Stateful wrappers
    // -------------------------

    /**
     * Moyenne mobile sur N échantillons (wrapper WPILib LinearFilter).
     */
    public static final class MovingAverage {
        private final LinearFilter filter;

        /**
         * @param taps nombre d'échantillons (doit être > 0)
         * @throws IllegalArgumentException si {@code taps <= 0}
         */
        public MovingAverage(int taps) {
            if (taps <= 0)
                throw new IllegalArgumentException("taps must be > 0");
            this.filter = LinearFilter.movingAverage(taps);
        }

        public double calculate(double x) {
            return filter.calculate(x);
        }

        public void reset() {
            filter.reset();
        }
    }

    /**
     * Filtre médian sur N échantillons (wrapper WPILib MedianFilter).
     */
    public static final class Median {
        private final MedianFilter filter;

        /**
         * @param taps taille de fenêtre (doit être > 0)
         * @throws IllegalArgumentException si {@code taps <= 0}
         */
        public Median(int taps) {
            if (taps <= 0)
                throw new IllegalArgumentException("taps must be > 0");
            this.filter = new MedianFilter(taps);
        }

        public double calculate(double x) {
            return filter.calculate(x);
        }

        public void reset() {
            filter.reset();
        }
    }

    /**
     * Limiteur de vitesse de variation (slew rate) (wrapper WPILib
     * SlewRateLimiter).
     */
    public static final class RateLimiter {
        private final SlewRateLimiter limiter;

        public RateLimiter(double ratePerSec) {
            this.limiter = new SlewRateLimiter(ratePerSec);
        }

        public double calculate(double target) {
            return limiter.calculate(target);
        }

        public void reset(double value) {
            limiter.reset(value);
        }
    }

    /**
     * Debounce d'un signal booléen (wrapper WPILib Debouncer).
     */
    public static final class DebouncedBoolean {
        private final Debouncer debouncer;

        public DebouncedBoolean(double seconds, Debouncer.DebounceType type) {
            this.debouncer = new Debouncer(seconds, type);
        }

        public boolean calculate(boolean x) {
            return debouncer.calculate(x);
        }
    }

    /**
     * Détecteur de fronts (rising/falling) sans dépendances.
     */
    public static final class Edge {
        private boolean prev = false;

        public boolean rising(boolean x) {
            boolean r = x && !prev;
            prev = x;
            return r;
        }

        public boolean falling(boolean x) {
            boolean f = !x && prev;
            prev = x;
            return f;
        }

        public void reset(boolean value) {
            prev = value;
        }
    }

    /**
     * Hold booléen sur une durée.
     */
    public static final class HoldBoolean {
        private final double holdSec;
        private boolean state = false;
        private double lastTrueTime = -1.0;

        public HoldBoolean(double holdSec) {
            this.holdSec = Math.max(0.0, holdSec);
        }

        public boolean calculate(boolean x) {
            double now = Timer.getFPGATimestamp();

            if (x) {
                state = true;
                lastTrueTime = now;
                return true;
            }

            if (state && lastTrueTime >= 0.0 && (now - lastTrueTime) < holdSec) {
                return true;
            }

            state = false;
            return false;
        }

        public void reset(boolean value) {
            state = value;
            lastTrueTime = value ? Timer.getFPGATimestamp() : -1.0;
        }
    }
}
