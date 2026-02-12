# FRC-2026 — Code Robot Équipe 3544 (WPILib Java)

Stack technique :
- **WPILib (Java, command-based)**
- **AdvantageKit / AdvantageScope** : logging + télémétrie
- **PathPlanner** : autos
- **PhotonVision / AprilTags** (multi-caméras) → **WPILib PoseEstimator** (fusion de pose)

Conçu pour le **shoot-on-the-move** (balistique dynamique), avec une séparation claire entre la **librairie mentor** (logique avancée, testable) et le code des **étudiants** (subsystems/commands).

---

## Structure du repo
```

src/main/java/frc
├─ mentor/ # librairie mentor/lead programmer (black boxes réutilisables)
│ ├─ logging/ # wrappers AdvantageKit + conventions
│ ├─ logic/ # balistique, dynamique, vision (pur/testable)
│ ├─ robot/ # records/classes mutables/config/pathplanner helpers
│ └─ utils/ # filtres, validation, erreurs, états machine
└─ robot/ # code robot (étudiants)
├─ subsystems/ # moteurs/capteurs/PID + contrôle mécanismes
├─ commands/ # driver controls, séquences, automation
└─ Constants.java # ports, IDs CAN, constantes

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

## Logging (AdvantageKit)

- Tous les helpers de logging sont dans `frc.mentor.logging`.
- But : les étudiants appellent une API simple (valeurs/événements/erreurs) sans se soucier du backend.
- Dans AdvantageScope on peut voir :
  - état robot + télémétrie
  - flags d’erreurs/status
  - timing/profiling

---

## Pipeline vision (multi-cam)

- Detections AprilTag via 4 caméras (PhotonVision).
- Mesures compensées en latence puis fusionnées dans une pose robot via WPILib PoseEstimator.
- La logique shooter consomme une mesure standardisée :
  - pose (optionnelle)
  - timestamp + latence
  - tag id(s)
  - confiance (0..1)

---

## Git workflow (obligatoire)

- Aucun push direct sur `main`.
- Travailler sur une branche, ouvrir une PR, obtenir 1 approbation (mentor ou lead-programmer selon le dossier).

Flow typique :

```bash
git checkout -b feature/mon-changement
git add -p
git commit -m "description du changement en quelques mots"
git push -u origin feature/mon-changement
```

Ensuite : ouvrir une Pull Request sur GitHub pour l'approbation par un mentor/lead programmer.

### CODEOWNERS / reviews

- `src/main/java/frc/mentor/**` : review mentor obligatoire
- `src/main/java/frc/robot/**` et `src/main/deploy/**` : mentor OU lead-programmer
- `docs/**` : programmer (ou lead-programmer/mentor)

---

## Conventions

- **Unités dans les noms :** `Deg`, `Mps`, `Degps`, `Volts`, `Sec`, etc.
- **Records pour le data :** utiliser `frc.mentor.robot.Records` pour standardiser les échanges entre couches.
- Garder `frc.robot` simple et lisible; mettre les logiques pures et complexes dans `frc.mentor`.

Exemples:
- hoodDeg = angle du hood en degrés
- turretDeg = angle de la turret en degrés
- vxMps = vitesse X en mètres/seconde
- omegaDegps = vitesse angulaire en deg/s
- accelMps2 = accélération en m/s²
- timestampSec = temps en secondes
- batteryVolts = tension en volts

---

## Notes

- Nouveau subsystem : le hardware reste dans le subsystem.
- Logique complexe à intégrer (exemple : calculs de balistique) : ça va généralement dans `frc.mentor`.

---
