## 🧑‍💻 Autori

- Ghena-Ionescu Alexandru
- Barbu Eduard
- Uta Mario-Ernest


# 🚗 Simulator de Conducere

Un simulator construit în Unity pentru a reproduce comportamentul unei mașini care navighează într-un mediu virtual folosind pathfinding (A*), evitarea obstacolelor, semafoare și un sistem realist de control al vehiculului.

Proiectul simulează un vehicul controlat de AI care se deplasează automat către ținte prestabilite, detectează obstacolele din față, respectă semafoarele și își ajustează direcția și viteza în mod dinamic în funcție de mediul înconjurător.

---

## 🧠 Funcționalități

- 🔁 **AI bazat pe mașină de stări** – Stări precum: Idle, Pathfind, Drive, Stop, Recover.
- 📍 **Pathfinding A\*** – Algoritm de calculare a traseului pe o hartă discretizată (grilă).
- 🚦 **Gestionare semafoare** – Vehiculul se oprește la roșu și pornește când semaforul devine verde.
- ⚙️ **Fizică realistă** – Control al vitezei, direcției, frânării și schimbării treptelor.
- 🔄 **Recuperare automată** – Mașina dă înapoi și se repoziționează dacă rămâne blocată.

---

## 🛠 Tehnologii utilizate

- **Unity** – motorul principal folosit pentru simulare și fizică.
- **C#** – limbajul în care sunt scrise toate scripturile.
- **Rigidbody & WheelCollider** – pentru mișcare realistă a vehiculului.
- **Gizmos și Debug** – pentru vizualizarea traseelor și a razelor de detecție în editor.

---

## 📂 Structura proiectului

- `CarAI.cs` – logica principală a AI-ului și tranzițiile între stări.
- `CarController.cs` – controlul fizic al mașinii (cuplu, frânare, direcție).
- `GridManager.cs` – generarea grilei pentru pathfinding.
- `TargetSelector.cs` – gestionarea punctelor de destinație.
- `Node.cs` – reprezentarea unui nod din rețeaua de pathfinding.

---

