package frc.mentor.utils;

import java.util.function.DoubleUnaryOperator;

/**
 * Outils de maths simples pour le robot.
 *
 * Idée générale:
 * - petites fonctions réutilisables (pas de mémoire interne)
 * - utiles partout (vision, shooter, swerve, etc.)
 *
 * Tout ici est "stateless": ça ne garde rien entre deux appels.
 */
public final class MathUtils {
  private MathUtils() {
  }
  /**
   * Force une valeur à rester dans une plage [lo, hi].
   *
   * Exemple:
   * - clamp(12, 0, 10) -> 10
   * - clamp(-2, 0, 10) -> 0
   *
   * @param x
   *          valeur à limiter
   * @param lo
   *          minimum
   * @param hi
   *          maximum
   * @return x ramené dans [lo, hi]
   */
  public static double clamp(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }

  /**
   * "Wrap" un angle (radians) pour qu'il reste entre -pi et +pi.
   *
   * Pourquoi?
   * Parce que les angles "bouclent": 181° c'est presque pareil que -179°.
   * Sans wrap, on peut avoir des énormes sauts et des calculs faux.
   *
   * @param a
   *          angle en radians
   * @return angle équivalent, entre (-pi, +pi]
   */
  public static double wrapRad(double a) {
    return Math.atan2(Math.sin(a), Math.cos(a));
  }

  /**
   * Différence d'angle la plus courte entre a et b (a - b), avec wrap.
   *
   * Exemple:
   * - a = 179°, b = -179° -> diff ~ 2° (pas 358°)
   *
   * @param a
   *          angle A (rad)
   * @param b
   *          angle B (rad)
   * @return différence signée la plus courte (rad)
   */
  public static double angleDiffRad(double a, double b) {
    return wrapRad(a - b);
  }

  /**
   * Interpole un angle de a vers b en prenant le chemin le plus court.
   *
   * t:
   * - 0.0 -> retourne a
   * - 1.0 -> retourne b
   * - 0.5 -> au milieu (dans le sens le plus court)
   *
   * @param a
   *          angle départ (rad)
   * @param b
   *          angle cible (rad)
   * @param t
   *          fraction (souvent 0..1)
   * @return angle interpolé (rad), wrapé
   */
  public static double angleLerpRad(double a, double b, double t) {
    double d = wrapRad(b - a);
    return wrapRad(a + d * t);
  }

  /**
   * Limite la vitesse de changement d'une valeur.
   *
   * Ça empêche de "sauter" trop vite.
   * Exemple: tu veux aller de 0 à 10, mais maxDelta=2:
   * - 0 -> 2 -> 4 -> 6 -> 8 -> 10
   *
   * @param curr
   *          valeur actuelle
   * @param target
   *          valeur désirée
   * @param maxDelta
   *          changement max autorisé (par appel)
   * @return nouvelle valeur (un pas vers target)
   */
  public static double rateLimit(double curr, double target, double maxDelta) {
    double d = target - curr;
    if (d > maxDelta)
      d = maxDelta;
    if (d < -maxDelta)
      d = -maxDelta;
    return curr + d;
  }

  /**
   * Wrap un angle en degrés dans (-180, 180].
   *
   * Ça sert quand tu veux comparer des angles en degrés (ex: blind spot turret),
   * sans te faire avoir par le wrap 180/-180.
   *
   * @param deg
   *          angle en degrés
   * @return angle équivalent wrapé en degrés (-180, 180]
   */
  public static double wrapDeg(double deg) {
    return Math.toDegrees(wrapRad(Math.toRadians(deg)));
  }

  /**
   * Vérifie si un angle (deg) est dans une bande [center - halfWidth, center + halfWidth],
   * en tenant compte du wrap -180/180.
   *
   * Si halfWidthDeg &lt;= 0, la bande est considérée comme désactivée.
   *
   * @param deg
   *          angle à tester (deg)
   * @param centerDeg
   *          centre de la bande (deg)
   * @param halfWidthDeg
   *          demi-largeur de la bande (deg)
   * @return true si deg est dans la bande
   */
  public static boolean inBandDeg(double deg, double centerDeg, double halfWidthDeg) {
    if (halfWidthDeg <= 0.0)
      return false;
    double d = wrapDeg(deg - centerDeg);
    return Math.abs(d) <= halfWidthDeg;
  }

  // -------------------------
  // minimisation
  // -------------------------

  /**
   * Résultat "Bracket": un petit intervalle [a, b] qui contient
   * probablement le minimum, plus le meilleur point trouvé.
   *
   * @param a
   *          borne gauche
   * @param b
   *          borne droite
   * @param xBest
   *          x qui a donné le plus petit f(x) pendant l'échantillonnage
   * @param fBest
   *          valeur f(xBest)
   */
  public static record Bracket(double a, double b, double xBest, double fBest) {
  }

  /**
   * Trouve un bracket [a, b] autour d'un minimum en testant n points.
   *
   * Comment ça marche:
   * - on teste f(x) sur une "ligne" de n points entre lo et hi
   * - on prend le point où f(x) est le plus petit
   * - on retourne un petit intervalle autour (point avant / point après)
   *
   * @param lo
   *          borne basse
   * @param hi
   *          borne haute
   * @param n
   *          nombre de tests (min 3)
   * @param f
   *          fonction qu'on veut minimiser
   * @return un Bracket [a,b] + meilleur point observé
   */
  public static Bracket bracketFromSamples(double lo, double hi, int n, DoubleUnaryOperator f) {
    int nn = Math.max(3, n);
    if (hi < lo) {
      double t = lo;
      lo = hi;
      hi = t;
    }

    double[] xs = new double[nn];
    double[] fs = new double[nn];

    for (int i = 0; i < nn; i++) {
      double x = lo + (hi - lo) * (i / (double) (nn - 1));
      xs[i] = x;
      fs[i] = f.applyAsDouble(x);
    }

    int k = 0;
    for (int i = 1; i < nn; i++) {
      if (fs[i] < fs[k])
        k = i;
    }

    double a, b;
    if (k == 0) {
      a = xs[0];
      b = xs[1];
    } else if (k == nn - 1) {
      a = xs[nn - 2];
      b = xs[nn - 1];
    } else {
      a = xs[k - 1];
      b = xs[k + 1];
    }

    return new Bracket(a, b, xs[k], fs[k]);
  }

  /**
   * Résultat d'une minimisation: le meilleur x et sa valeur f(x).
   *
   * @param x
   *          meilleur x trouvé
   * @param fx
   *          f(x)
   */
  public static record MinResult(double x, double fx) {
  }

  /**
   * Cherche le minimum d'une fonction sur un intervalle [a, b]
   * avec la méthode "section dorée".
   *
   * Important:
   * - ça marche super bien si f(x) descend puis remonte (un seul creux)
   * - plus iters est grand, plus c'est précis (mais plus lent)
   *
   * @param f
   *          fonction à minimiser
   * @param a
   *          borne gauche
   * @param b
   *          borne droite
   * @param iters
   *          nombre d'itérations (min 1)
   * @return MinResult(x, f(x)) pour le meilleur point trouvé
   */
  public static MinResult goldenSectionMinimize(DoubleUnaryOperator f, double a, double b, int iters) {
    if (b < a) {
      double t = a;
      a = b;
      b = t;
    }

    if (Math.abs(b - a) < 1e-12) {
      double x = 0.5 * (a + b);
      return new MinResult(x, f.applyAsDouble(x));
    }

    final double phi = 0.6180339887498949; // ratio fixe magique de la méthode
    double c = b - phi * (b - a);
    double d = a + phi * (b - a);
    double fc = f.applyAsDouble(c);
    double fd = f.applyAsDouble(d);

    int N = Math.max(1, iters);
    for (int i = 0; i < N; i++) {
      if (fc < fd) {
        b = d;
        d = c;
        fd = fc;
        c = b - phi * (b - a);
        fc = f.applyAsDouble(c);
      } else {
        a = c;
        c = d;
        fc = fd;
        d = a + phi * (b - a);
        fd = f.applyAsDouble(d);
      }
    }

    return (fc < fd) ? new MinResult(c, fc) : new MinResult(d, fd);
  }
}
