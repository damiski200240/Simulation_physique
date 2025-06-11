# Simulation Physique – Moteur CC & Contrôle PID

Ce projet est issu d’un travail universitaire réalisé dans le cadre du Master SAR (Systèmes Avancés et Robotique) à Sorbonne Université. Il porte sur la modélisation, la simulation et la commande d’un moteur à courant continu (CC) en Python, avec des applications à la robotique mobile (type TurtleBot).

## 📌 Objectifs

- Modéliser un moteur CC à partir de ses équations mécaniques et électriques
- Implémenter une régulation PID en vitesse puis en position
- Intégrer le moteur dans un système robotique simulé (Turtlebot)
- Comparer les approches cinématiques idéales vs physiques (avec dynamique moteur)
- Illustrer le comportement du système avec des simulations et des visualisations

## 🛠️ Fonctionnalités principales

- **Classe `moteurCC`** : simulation du comportement moteur (avec ou sans inductance)
- **Contrôleurs PID** : pour la vitesse (`ControlPid_vitesse`) et la position (`ControlPid_position`)
- **Simulateur de robots `Turtle` / `TurtlePID`** : déplacement vers une cible avec ou sans contrôle moteur
- **Modèle cinématique inverse pour robots différentiels**
- **Visualisation graphique** : trajectoires, réponses temporelles, comparaisons




## 🔍 Aperçu des résultats

Quelques exemples :
- Réponse du moteur à un échelon (analytique vs simulation)
- Régulation PID : influence des gains
- Comparaison TurtleBot idéal vs avec moteurs simulés

![Comparaison Turtle vs TurtlePID](./5sec.png)

## 🚀 Lancer une simulation

1. Cloner le dépôt :
```bash
git clone https://github.com/damiski200240/Simulation_physique.git
cd Simulation_physique
````

2. Exécuter un des fichiers principaux, par exemple :

```bash
python Turtle_traj.py
```

> Assurez-vous d’avoir `matplotlib` et `numpy` installés :

```bash
pip install matplotlib numpy
```

## 🧠 Auteurs

* **Imad GHOMARI** – [LinkedIn](www.linkedin.com/in/ismail-ghomari)
* Master 1 SAR – Sorbonne Université

## 📄 Rapport complet

Un rapport PDF détaillé est disponible dans ce dépôt : [Rapport\_Projet\_Simulation.pdf](./Rapport_Projet_Simulation.pdf)


