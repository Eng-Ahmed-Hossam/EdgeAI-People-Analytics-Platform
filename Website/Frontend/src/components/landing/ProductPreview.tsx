'use client';

import { useRef } from 'react';
import { motion, useInView } from 'framer-motion';
import { TrendingUp, Users, DollarSign, AlertCircle } from 'lucide-react';
import type { LucideIcon } from 'lucide-react';

interface Metric {
  icon: LucideIcon;
  label: string;
  value: string;
  unit: string;
  change: string;
  /* color: kept as hardcoded Tailwind value where no exact token exists — see comments */
  color: string;
}

const metrics: Metric[] = [
  { icon: TrendingUp,  label: 'Feasibility Score', value: '87',       unit: '/100', change: '+12',        color: 'text-verdict-strong' },
  { icon: Users,       label: 'Daily Foot Traffic', value: '3,247',    unit: 'avg',  change: '+18%',       color: 'text-accent-data'    },
  { icon: DollarSign,  label: 'Revenue Estimate',   value: '$42K',     unit: '/mo',  change: 'Medium risk', color: 'text-accent-muted'   },
  { icon: AlertCircle, label: 'Risk Level',          value: 'Moderate', unit: '',     change: 'Stable',      color: 'text-status-low'     },
];

const barHeights = [40, 55, 45, 70, 65, 80, 75, 85, 78, 90, 88, 95];

function Row({ label, value, highlight, accent }: {
  label: string;
  value: string;
  highlight?: boolean;
  accent?: boolean;
}) {
  return (
    <div className="flex justify-between">
      <span className="text-ink-muted">{label}</span>
      {/* Financial values use font-mono for tabular alignment */}
      <span className={`font-mono ${highlight ? 'text-verdict-strong' : accent ? 'text-accent-data' : 'text-ink-strong'}`}>
        {value}
      </span>
    </div>
  );
}

export function ProductPreview() {
  const ref = useRef<HTMLDivElement>(null);
  const isInView = useInView(ref, { once: true, amount: 0.2 });

  return (
    <section id="product-preview" className="py-32 bg-page">
      <div className="max-w-7xl mx-auto px-6">
        <motion.div
          ref={ref}
          initial={{ opacity: 0, y: 30 }}
          animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
          transition={{ duration: 0.8 }}
          className="text-center mb-16"
        >
          <h2 className="mb-4">Production-Ready Intelligence</h2>
          <p className="text-xl text-ink-muted max-w-3xl mx-auto">
            FEASIX delivers a unified dashboard combining real-time activity, financial
            projections, and risk analysis.
          </p>
        </motion.div>

        {/* Metric cards */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-12">
          {metrics.map((metric, index) => (
            <motion.div
              key={metric.label}
              initial={{ opacity: 0, y: 40 }}
              animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 40 }}
              transition={{ duration: 0.6, delay: index * 0.1 }}
              className="bg-surface border border-raised rounded-xl p-6 hover:border-strong transition-all"
            >
              <div className="flex items-start justify-between mb-4">
                <div className={`w-10 h-10 rounded-lg bg-raised flex items-center justify-center ${metric.color}`}>
                  <metric.icon className="w-5 h-5" />
                </div>
                <span className="text-xs text-ink-faint">{metric.label}</span>
              </div>
              <div className="mb-2">
                {/* Primary metric values use font-mono for tabular treatment */}
                <span className="text-3xl font-mono font-semibold text-ink-strong">{metric.value}</span>
                {metric.unit && <span className="text-ink-faint ml-1 text-sm">{metric.unit}</span>}
              </div>
              <div className="text-sm text-ink-muted">{metric.change}</div>
            </motion.div>
          ))}
        </div>

        {/* Chart preview */}
        <motion.div
          initial={{ opacity: 0, y: 40 }}
          animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 40 }}
          transition={{ duration: 0.8, delay: 0.5 }}
          className="bg-surface border border-raised rounded-xl p-8"
        >
          <div className="flex items-center justify-between mb-8">
            <div>
              <h3 className="mb-1">Foot Traffic Analysis</h3>
              <p className="text-sm text-ink-muted">Last 30 days</p>
            </div>
          </div>

          <div className="relative h-64">
            <div className="absolute inset-0 flex items-end justify-between gap-2">
              {barHeights.map((height, i) => (
                <motion.div
                  key={i}
                  /* from-accent-data to-accent: cyan→blue gradient for bars.
                     Original was from-cyan-600 to-blue-600; cyan-600 ≠ accent-data (cyan-400).
                     Shifted to accent-data (cyan-400) — slightly brighter; flagged. */
                  className="flex-1 bg-linear-to-t from-accent-data to-accent rounded-t opacity-80"
                  initial={{ height: 0 }}
                  animate={isInView ? { height: `${height}%` } : { height: 0 }}
                  transition={{ duration: 0.8, delay: 0.7 + i * 0.05 }}
                />
              ))}
            </div>

            {[0, 25, 50, 75, 100].map((pos) => (
              <div
                key={pos}
                className="absolute left-0 right-0 border-t border-raised"
                style={{ bottom: `${pos}%` }}
              >
                {/* text-ink-faint (slate-500) vs original slate-600 — one step lighter; flagged */}
                <span className="absolute -left-12 -top-2 text-xs font-mono text-ink-faint">{pos}%</span>
              </div>
            ))}
          </div>
        </motion.div>

        {/* Info panels */}
        <div className="grid md:grid-cols-2 gap-6 mt-8">
          <motion.div
            initial={{ opacity: 0, x: -20 }}
            animate={isInView ? { opacity: 1, x: 0 } : { opacity: 0, x: -20 }}
            transition={{ duration: 0.8, delay: 0.8 }}
            className="bg-surface/50 border border-raised rounded-xl p-6"
          >
            <h4 className="mb-3 font-medium">Financial Projection</h4>
            <div className="space-y-2 text-sm">
              <Row label="Monthly Revenue Range" value="$38K – $47K" />
              <Row label="Break-Even Timeline"   value="8–12 months" />
              <Row label="ROI Estimate"           value="22–28%"      highlight />
            </div>
          </motion.div>

          <motion.div
            initial={{ opacity: 0, x: 20 }}
            animate={isInView ? { opacity: 1, x: 0 } : { opacity: 0, x: 20 }}
            transition={{ duration: 0.8, delay: 0.8 }}
            className="bg-surface/50 border border-raised rounded-xl p-6"
          >
            <h4 className="mb-3 font-medium">Location Insights</h4>
            <div className="space-y-2 text-sm">
              <Row label="Population Density" value="High" />
              <Row label="Competition Level"  value="Moderate" />
              <Row label="Traffic Peak Hours" value="12pm–2pm, 5pm–7pm" accent />
            </div>
          </motion.div>
        </div>
      </div>
    </section>
  );
}
