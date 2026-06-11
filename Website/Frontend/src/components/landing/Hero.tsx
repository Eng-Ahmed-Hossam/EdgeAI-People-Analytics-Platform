'use client';

import { motion } from 'framer-motion';

function scrollTo(sectionId: string) {
  const el = document.getElementById(sectionId);
  if (el) el.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function Indicator({ label, color }: { label: string; color: string }) {
  return (
    <div className="flex items-center gap-2">
      <div className={`w-2 h-2 rounded-full animate-pulse ${color}`} />
      <span>{label}</span>
    </div>
  );
}

export function Hero() {
  return (
    <section className="relative min-h-screen flex items-center justify-center overflow-hidden">
      {/* Background stack — CSS-only, no image dependency */}
      <div className="absolute inset-0 bg-page">
        {/* Slow-drifting radial glows — decorative animation; inline values intentional */}
        <motion.div
          className="absolute inset-0"
          animate={{
            background: [
              'radial-gradient(ellipse 80% 55% at 15% 40%, rgba(30,64,175,0.18) 0%, transparent 65%), radial-gradient(ellipse 55% 40% at 85% 65%, rgba(6,182,212,0.12) 0%, transparent 65%)',
              'radial-gradient(ellipse 80% 55% at 65% 25%, rgba(30,64,175,0.18) 0%, transparent 65%), radial-gradient(ellipse 55% 40% at 25% 75%, rgba(6,182,212,0.12) 0%, transparent 65%)',
              'radial-gradient(ellipse 80% 55% at 15% 40%, rgba(30,64,175,0.18) 0%, transparent 65%), radial-gradient(ellipse 55% 40% at 85% 65%, rgba(6,182,212,0.12) 0%, transparent 65%)',
            ],
          }}
          transition={{ duration: 20, repeat: Infinity, ease: 'easeInOut' }}
        />

        {/* Fine grid texture — decorative; inline backgroundImage intentional */}
        <div
          className="absolute inset-0 opacity-[0.035]"
          style={{
            backgroundImage:
              'linear-gradient(rgba(148,163,184,1) 1px, transparent 1px), linear-gradient(90deg, rgba(148,163,184,1) 1px, transparent 1px)',
            backgroundSize: '60px 60px',
          }}
          aria-hidden="true"
        />

        <div className="absolute inset-0 bg-linear-to-b from-transparent via-page/40 to-page" />

        {/* Animated data-point dots */}
        {Array.from({ length: 15 }).map((_, i) => (
          <motion.div
            key={i}
            className="absolute w-1 h-1 bg-accent-data rounded-full"
            style={{ left: `${(i * 17) % 100}%`, top: `${(i * 29) % 100}%` }}
            animate={{ opacity: [0.2, 0.8, 0.2], scale: [1, 1.5, 1] }}
            transition={{ duration: 5, repeat: Infinity, delay: i * 0.2 }}
          />
        ))}
      </div>

      {/* Content */}
      <div className="relative z-10 max-w-5xl mx-auto px-6 text-center">
        <motion.h1
          className="text-6xl font-semibold mb-6 text-ink-strong"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
        >
          Make Business Decisions With Confidence —{' '}
          <span className="text-accent-muted">Before You Invest</span>
        </motion.h1>

        <motion.p
          className="text-xl text-ink-body mb-12 max-w-3xl mx-auto leading-relaxed"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.2 }}
        >
          FEASIX combines real-world location activity, hardware sensing, and financial
          modeling to deliver objective business feasibility assessments.
        </motion.p>

        <motion.div
          className="flex items-center justify-center gap-4"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.4 }}
        >
          <button
            onClick={() => scrollTo('map-preview')}
            className="px-8 py-4 bg-accent hover:bg-accent-hover text-ink-strong rounded-lg transition-all font-medium text-lg shadow-lg shadow-accent-dim/50"
          >
            Explore Feasibility on the Map
          </button>
          <button
            onClick={() => scrollTo('how-it-works')}
            className="px-8 py-4 bg-raised/50 hover:bg-raised text-ink-strong rounded-lg transition-all font-medium text-lg border border-strong"
          >
            See How FEASIX Works
          </button>
        </motion.div>

        <motion.div
          className="mt-20 flex items-center justify-center gap-16 text-sm text-ink-muted"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 1, delay: 1 }}
        >
          <Indicator label="Live Evidence" color="bg-accent-data"  />
          <Indicator label="Edge Hardware"  color="bg-accent-muted" />
          <Indicator label="Decision Intelligence"   color="bg-accent-muted" />
        </motion.div>
      </div>

      <div className="absolute bottom-0 left-0 right-0 h-32 bg-linear-to-t from-page to-transparent" />
    </section>
  );
}
