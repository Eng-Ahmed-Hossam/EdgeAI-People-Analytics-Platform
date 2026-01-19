# Edge AI Location Intelligence Platform
### Profit Prediction & Feasibility Scoring using Edge Data

---

## 📌 Overview

The **Edge AI Location Intelligence Platform** is a full **data engineering and analytics system** designed to **predict business revenue, profit, and feasibility** using **real-time Edge AI signals**, business metadata, and exogenous factors such as weather, events, and promotions.

This project follows **industry-grade data architecture principles**:
- Deterministic ETL pipelines
- Clean separation between raw data, derived data, features, and predictions
- ML-ready feature store
- Data quality governance
- Scalable and auditable design

It is suitable for:
- Graduation projects
- Research prototypes
- Production-ready system foundations

---

## 🧠 System Architecture

Raw Data (Edge / Business / External) <br>
↓ <br>
Deterministic Aggregations (Hourly / Daily) <br>
↓ <br>
Behavioral Intelligence (Trends & Volatility) <br>
↓ <br>
Feature Store (ML-Ready) <br>
↓ <br>
Prediction Output (Revenue / Profit / Feasibility) <br>
↓ <br>
Data Quality & Monitoring <br>


---

## 📁 Repository Structure

scripts/ <br>
│ <br>
├── 00_admin/ # Operational & admin scripts <br>
│ ├── 00_reset_edge_ai_db.sql <br>
│ ├── 01_reset_database_data.sql <br>
│ └── 02_optimize_indexes.sql <br>
│ <br>
├── 01_schema/ # Schema & table definitions <br>
│ ├── 03_create_schema.sql <br>
│ └── 04_create_relationships.sql <br>
│ <br>
├── 02_base_tables/ # Raw & source data (immutable) <br>
│ ├── 05_location_metadata.sql <br>
│ ├── 06_business_profile.sql <br>
│ ├── 07_realtime_metrics.sql <br>
│ ├── 08_weather.sql <br>
│ ├── 09_events.sql <br>
│ ├── 10_promotions.sql <br>
│ └── 11_daily_revenue.sql <br>
│ <br>
├── 03_derived_tables/ # Deterministic aggregations <br>
│ ├── 12_hourly_fpga_metrics.sql <br>
│ ├── 13_daily_fpga_metrics.sql <br>
│ ├── 14_behavior_trends.sql <br>
│ └── 15_operational_efficiency.sql <br>
│ <br>
├── 04_feature_layer/ # Model-ready data <br>
│ └── 16_feature_store.sql <br>
│ <br>
├── 05_predictions/ # Model metadata & outputs <br>
│ ├── 17_model_metadata.sql <br>
│ └── 18_prediction_output.sql <br>
│ <br>
├── 06_data_quality/ # Monitoring & governance <br>
│ └── 19_data_quality_log.sql <br>
│ <br>
├── 07_views/ # BI & analytics views (read-only) <br>
│ └── 20_create_views.sql <br>
│ <br>
├── 08_CTE <br>
│ └── 21_context_feature_expansion.sql <br>
│ <br>
└── README.md <br>


---

## 🗄️ Data Layers Explained

### 1️⃣ Base Tables (Source Layer)
Raw, immutable data ingested from:
- Edge AI sensors (footfall, dwell, traffic)
- Business profiles and costs
- Weather data
- Events and promotions
- POS daily revenue (ML labels)

> These tables are **never manually aggregated or overwritten**.

---

### 2️⃣ Derived Tables (Aggregation Layer)
Deterministic transformations:
- Hourly FPGA metrics
- Daily FPGA metrics
- Behavioral trends (growth, volatility)
- Operational efficiency & profit margins

> Fully rebuildable and replay-safe.

---

### 3️⃣ Feature Store (ML Layer)
A centralized, model-agnostic feature repository:
- One row per business per day
- Time-aligned features
- Feature vector hashing for reproducibility
- No data leakage

---

### 4️⃣ Prediction Output (Inference Layer)
Stores ML inference results:
- Predicted revenue
- Predicted profit
- Feasibility score (0–100)
- Confidence score
- Model version tracking

> Models are trained **outside the database**.

---

### 5️⃣ Data Quality & Governance
Daily monitoring of:
- Completeness
- Missing intervals
- Sensor anomalies
- Label sanity

Ensures **trustworthy analytics and ML outputs**.