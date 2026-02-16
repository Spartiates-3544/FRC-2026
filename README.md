```md
# FRC-2026 — Code Robot Équipe 3544 (WPILib Java)

Stack technique :

- **WPILib (Java, command-based)**
- **DogLog + AdvantageScope** : logs + télémétrie
- **PathPlanner** : autos
- **PhotonVision / AprilTags** (multi-caméras) → **WPILib PoseEstimator** (fusion de pose)

Objectif : robot prêt pour le **shoot-on-the-move** (balistique + latency), avec une séparation claire entre :

- **`frc.mentor/`** : librairie mentor (logique avancée, testable, réutilisable d'années en années)
- **`frc.robot/`** : code étudiant (subsystems/commands simple, lisible, débuggable et tunable)

---

## Structure du repo
```

src/main/java/frc
├─ mentor/ # librairie mentor / lead programmer (black boxes réutilisables)
│ ├─ logging/ # logging DogLog + helpers
│ │ └─ ExtendedLogger.java
│ ├─ logic/ # balistique, dynamique, visée (pur/testable)
│ │ ├─ BallisticSolver.java
│ │ ├─ ShooterAim.java
│ │ ├─ ShooterLogic.java
│ │ └─ ShooterTuning.java
│ ├─ robot/
│ │ ├─ Records.java
│ │ ├─ Config.java
│ │ └─ Tunables.java
│ └─ utils/ # filtres, maths, helpers
│ ├─ Filters.java
│ └─ MathUtils.java
└─ robot/ # code robot (étudiants)
├─ subsystems/ # moteurs/capteurs/PID + contrôle mécanismes
├─ commands/ # driver controls, séquences, automation
├─ RobotContainer.java
└─ Constants.java # paramètres par défaut des sous-systèmes

````

---

## Démarrage (dev)

### Prérequis
- WPILib installé pour la saison (outils Java)
- Cloner le repo
- Ouvrir avec **WPILib VS Code**

### Build / vérification
```bash
./gradlew build
````

### Générer et ouvrir les JavaDocs

```bash
./gradlew javadoc
xdg-open build/docs/javadoc/index.html
```

---

## Logging (DogLog + AdvantageScope)

On utilise **DogLog** comme API de logging (et ça s’affiche dans **AdvantageScope** et autres dashboards connectés aux Network Tables).

### 1) Logger “simple” pour les élèves : `ExtendedLogger`

`ExtendedLogger` wrap DogLog et permet l’auto-logging via annotation.

**Pattern :**

- Tu mets `@ExtendedLogger.LoggableField(path="...")` sur des fields
- Tu appelles `ExtendedLogger.registerInstance(this)` une fois (dans le constructeur)
- Tu appelles `ExtendedLogger.run()` une fois par loop (robotPeriodic)

Exemple minimal :

```java
public final class ClimberSubsystem extends SubsystemBase {

  @ExtendedLogger.LoggableField(path="Climber/cmdVolts")
  private double cmdVolts = 0.0;

  public ClimberSubsystem() {
    ExtendedLogger.registerInstance(this);
  }

  @Override
  public void periodic() {
    // update ta variable...
  }
}
```

### 2) Appel global (obligatoire)

Dans `Robot.robotPeriodic()` :

```java
@Override
public void robotPeriodic() {
  Tunables.run();                   // update des valeurs tunables (NetworkTables)
  frc.mentor.logging.ExtendedLogger.run(); // log DogLog (auto-logging)
  CommandScheduler.getInstance().run();
}
```

### 3) Où voir les logs ?

Dans **AdvantageScope**, tu verras tes clés comme :

- `Climber/cmdVolts`
- `DriveState/Pose`
- `Shooter/solver_ms`
  etc.

---

## Tunables (NetworkTables /Tuning)

On a une couche simple pour tuner des valeurs **en live** via NetworkTables.

### Layout des clés

Toutes les valeurs tunables vont sous :

```
/Tuning/<key>
Ex: /Tuning/Drive/maxSpeed
Ex: /Tuning/Shooter/slipFactor
Ex: /Tuning/Climber/maxVolts
```

### 1) Version user friendly (annotation) : `@TunableNum/@TunableBool/@TunableStr`

Tu mets l’annotation directement sur un field, et `Tunables.run()` s’occupe de le mettre à jour.

```java
@Tunables.TunableNum(key="Climber/maxVolts", def=8.0, hz=5, clamp=true, min=0, max=12)
private double maxVolts = 8.0;

public ClimberSubsystem() {
  Tunables.registerInstance(this);
}
```

### Où modifier ces valeurs ?

Dans n’importe quel dashboard qui voit NetworkTables :

- Shuffleboard
- AdvantageScope (onglet NetworkTables)

---

## Vision (multi-cam) — aperçu (TODO)

- Detections AprilTag via caméras (PhotonVision)
- Mesures compensées en latence
- Fusion dans un pose estimator WPILib
- Le shooter consomme ensuite une pose standardisée (pose + timestamp + confiance)

_(Les détails d’implémentation viendront dans `frc.mentor.vision` ou bien dans `frc.robot.vision`.)_

---

## Utiliser la librairie mentor — exemples

### A) ShooterAim “mode simple” (recommandé)

```java
ShooterAim.Loop aim = new ShooterAim.Loop(Constants.Shooter.defaultParams());

ShooterAim.AimResult out = aim.update(robotNow, actNow, targetXYZ, turretYawNowRad);

// out.cmd() -> setpoints turret/rpm/hood
// out.ok()  -> autoriser feeder/indexer
```

### B) Logging automatique d’un record (exploded fields)

Si tu log un `record`, `ExtendedLogger` le “split” en sous-champs :

- `ShooterState/rpm`
- `ShooterState/hoodDeg`
- etc.

---

## Git workflow (obligatoire)

- Aucun push direct sur `main`
- Travailler sur une branche, faire une PR, obtenir 1 approbation

Flow typique :

```bash
git checkout -b mon-changement
git add -p
git commit -m "description simple du changement"
git push -u origin mon-changement
```

Ensuite : ouvrir une Pull Request sur GitHub.

### CODEOWNERS / reviews

- `src/main/java/frc/mentor/**` : review mentor obligatoire
- `src/main/java/frc/robot/**` + `src/main/deploy/**` : mentor OU lead-programmer
- `docs/**` : programmer (ou lead-programmer/mentor)

---

## Conventions

- **Unités dans les noms** : `Deg`, `Mps`, `Volts`, `Sec`, etc.
- **Commentaires en français et code en anglais** : Mieux pour la cohérence entre les librairies anglophones
- **Records pour le data** : utiliser `frc.mentor.robot.Records` pour standardiser les échanges

Exemples :

- `hoodDeg`
- `turretYawRelRad`
- `vxMps`, `omegaRadps`
- `timestampSec`
- `batteryVolts`
