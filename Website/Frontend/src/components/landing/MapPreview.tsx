'use client';

import { useRef, useState } from 'react';
import { motion, AnimatePresence, useInView } from 'framer-motion';
import { MapPin } from 'lucide-react';

interface Location {
  id: number;
  x: number;
  y: number;
  score: number;
  name: string;
  status: 'high' | 'medium' | 'low';
}

const locations: Location[] = [
  { id: 1, x: 25, y: 30, score: 87, name: 'Downtown Core',    status: 'high'   },
  { id: 2, x: 45, y: 45, score: 72, name: 'Tech District',    status: 'medium' },
  { id: 3, x: 65, y: 35, score: 91, name: 'Financial Hub',    status: 'high'   },
  { id: 4, x: 35, y: 60, score: 64, name: 'Suburban Plaza',   status: 'medium' },
  { id: 5, x: 75, y: 55, score: 58, name: 'Retail Strip',     status: 'low'    },
  { id: 6, x: 55, y: 25, score: 79, name: 'University Area',  status: 'medium' },
];

/* Literal class strings — Tailwind scans these at build time.
   Values correspond to --status-high/med/low tokens in globals.css. */
const statusClasses: Record<Location['status'], string> = {
  high:   'bg-green-500 border-green-400',
  medium: 'bg-cyan-500 border-cyan-400',
  low:    'bg-orange-500 border-orange-400',
};

function Legend({ color, label }: { color: 'green' | 'cyan' | 'orange'; label: string }) {
  const dotClasses: Record<string, string> = {
    green:  'bg-green-500',
    cyan:   'bg-cyan-500',
    orange: 'bg-orange-500',
  };
  return (
    <div className="flex items-center gap-2 text-sm text-ink-muted">
      <div className={`w-3 h-3 rounded-full ${dotClasses[color]}`} />
      {label}
    </div>
  );
}

// public/map-base.avif — correctly labeled AVIF asset (was map-base.png).
// MapPreview uses a CSS gradient background; the asset is available if needed.
export function MapPreview() {
  const ref = useRef<HTMLDivElement>(null);
  const isInView = useInView(ref, { once: true, amount: 0.2 });
  const [hoveredId, setHoveredId] = useState<number | null>(null);

  return (
    <section id="map-preview" className="py-32 bg-linear-to-b from-surface to-page">
      <div className="max-w-7xl mx-auto px-6">
        <motion.div
          ref={ref}
          initial={{ opacity: 0, y: 30 }}
          animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
          transition={{ duration: 0.8 }}
          className="text-center mb-16"
        >
          <h2 className="mb-4">Explore Feasibility Anywhere</h2>
          <p className="text-xl text-ink-muted max-w-3xl mx-auto">
            A conceptual preview of how FEASIX evaluates business feasibility across
            locations using spatial intelligence.
          </p>
        </motion.div>

        <motion.div
          initial={{ opacity: 0, y: 40 }}
          animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 40 }}
          transition={{ duration: 0.8, delay: 0.3 }}
          className="relative bg-surface border border-raised rounded-2xl p-8 overflow-hidden"
        >
          {/* Grid texture — #3b82f6 in inline style; decorative, not tokenizable */}
          <div className="absolute inset-0 opacity-5" aria-hidden="true">
            <div
              className="w-full h-full"
              style={{
                backgroundImage:
                  'linear-gradient(to right, #3b82f6 1px, transparent 1px), linear-gradient(to bottom, #3b82f6 1px, transparent 1px)',
                backgroundSize: '40px 40px',
              }}
            />
          </div>

          {/* Legend */}
          <div className="relative z-10 mb-6 flex items-center justify-between">
            <h3 className="">Metropolitan Feasibility Overview</h3>
            <div className="flex items-center gap-6">
              <Legend color="green"  label="High"   />
              <Legend color="cyan"   label="Medium" />
              <Legend color="orange" label="Lower"  />
            </div>
          </div>

          {/* Map area */}
          <div className="relative z-10 h-125 bg-page/80 rounded-xl border border-raised overflow-hidden">
            {/* Animated glow — decorative; inline radial-gradient values intentional */}
            <motion.div
              className="absolute inset-0 opacity-20"
              animate={{
                background: [
                  'radial-gradient(circle at 30% 40%, #3b82f6 0%, transparent 50%)',
                  'radial-gradient(circle at 70% 60%, #06b6d4 0%, transparent 50%)',
                  'radial-gradient(circle at 30% 40%, #3b82f6 0%, transparent 50%)',
                ],
              }}
              transition={{ duration: 10, repeat: Infinity, ease: 'linear' }}
            />

            {/* Location pins */}
            {locations.map((loc, index) => (
              <motion.div
                key={loc.id}
                className="absolute"
                style={{ left: `${loc.x}%`, top: `${loc.y}%`, transform: 'translate(-50%, -50%)' }}
                initial={{ opacity: 0, scale: 0 }}
                animate={isInView ? { opacity: 1, scale: 1 } : {}}
                transition={{ duration: 0.6, delay: 0.5 + index * 0.1 }}
                onMouseEnter={() => setHoveredId(loc.id)}
                onMouseLeave={() => setHoveredId(null)}
              >
                {/* Pulse halo */}
                <motion.div
                  className={`absolute inset-0 rounded-full ${statusClasses[loc.status]} blur-xl`}
                  animate={{ scale: [1, 2, 1], opacity: [0.5, 0, 0.5] }}
                  transition={{ duration: 3, repeat: Infinity }}
                />

                {/* Dot */}
                <div
                  className={`relative w-4 h-4 rounded-full border ${statusClasses[loc.status]} transition-transform ${
                    hoveredId === loc.id ? 'scale-150' : ''
                  }`}
                />

                {/* Tooltip */}
                <AnimatePresence>
                  {hoveredId === loc.id && (
                    <motion.div
                      initial={{ opacity: 0, y: 10 }}
                      animate={{ opacity: 1, y: 0 }}
                      exit={{ opacity: 0, y: 10 }}
                      className="absolute top-full left-1/2 -translate-x-1/2 mt-4 bg-surface border border-strong rounded-lg p-4 shadow-xl z-20 min-w-35"
                    >
                      <div className="flex items-center gap-2 mb-1">
                        <MapPin className="w-4 h-4 text-accent-data shrink-0" />
                        <span className="text-ink-strong font-medium text-sm">{loc.name}</span>
                      </div>
                      <div className="text-xs text-ink-muted">Feasibility</div>
                      {/* Score value: font-mono for tabular number treatment */}
                      <div className="text-2xl font-mono font-semibold text-ink-strong">{loc.score}/100</div>
                    </motion.div>
                  )}
                </AnimatePresence>
              </motion.div>
            ))}
          </div>

          {/* Footer note */}
          <div className="relative z-10 mt-6 flex items-center justify-between">
            <span className="text-xs text-ink-faint">
              Conceptual preview — full interactive map available in product
            </span>
            <span className="text-xs text-ink-faint italic">
              Interactive map coming soon
            </span>
          </div>
        </motion.div>
      </div>
    </section>
  );
}
