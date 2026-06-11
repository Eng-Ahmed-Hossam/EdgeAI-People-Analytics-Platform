'use client';

import { useRef } from 'react';
import { motion, useInView } from 'framer-motion';
import { ArrowRight, Map } from 'lucide-react';
import Link from 'next/link';

function scrollTo(sectionId: string) {
  const el = document.getElementById(sectionId);
  if (el) el.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

export function FinalCTA() {
  const ref = useRef<HTMLDivElement>(null);
  const isInView = useInView(ref, { once: true, amount: 0.3 });

  return (
    <section className="py-32 bg-linear-to-b from-surface to-page">
      <div className="max-w-7xl mx-auto px-6">
        <motion.div
          ref={ref}
          initial={{ opacity: 0, y: 40 }}
          animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 40 }}
          transition={{ duration: 0.8 }}
          /* from-accent-dim to-surface: deep blue → surface gradient.
             Original border-blue-900/50 → border-accent-dim/70; close enough. */
          className="relative bg-linear-to-br from-accent-dim to-surface border border-accent-dim/70 rounded-2xl p-16 text-center overflow-hidden"
        >
          {/* Grid texture — #3b82f6 in inline style; decorative, not tokenizable */}
          <div className="absolute inset-0 opacity-10" aria-hidden="true">
            <div
              className="w-full h-full"
              style={{
                backgroundImage:
                  'linear-gradient(to right, #3b82f6 1px, transparent 1px), linear-gradient(to bottom, #3b82f6 1px, transparent 1px)',
                backgroundSize: '60px 60px',
              }}
            />
          </div>

          {/* Animated glow — decorative */}
          <motion.div
            className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-96 h-96 bg-accent rounded-full blur-3xl opacity-20"
            animate={{ scale: [1, 1.2, 1], opacity: [0.2, 0.3, 0.2] }}
            transition={{ duration: 4, repeat: Infinity, ease: 'easeInOut' }}
          />

          <div className="relative z-10">
            <motion.div
              initial={{ opacity: 0, scale: 0.9 }}
              animate={isInView ? { opacity: 1, scale: 1 } : { opacity: 0, scale: 0.9 }}
              transition={{ duration: 0.8, delay: 0.2 }}
              className="inline-flex items-center justify-center w-16 h-16 rounded-full bg-accent/20 border border-accent/30 mb-8"
            >
              <Map className="w-8 h-8 text-accent-muted" />
            </motion.div>

            <h2 className="mb-6 max-w-3xl mx-auto">
              Ready to Make Data-Backed Business Decisions?
            </h2>

            <p className="text-xl text-ink-body mb-10 max-w-2xl mx-auto leading-relaxed">
              Explore FEASIX&#39;s interactive map and discover feasibility intelligence for
              locations across your target market. Start with confidence.
            </p>

            <div className="flex items-center justify-center gap-4">
              <Link
                href="/workspace"
                className="inline-flex items-center gap-2 px-8 py-4 bg-accent hover:bg-accent-hover text-ink-strong rounded-lg transition-all font-medium text-lg shadow-lg shadow-accent-dim/50"
              >
                Explore the Map
                <ArrowRight className="w-5 h-5" />
              </Link>

              <button
                onClick={() => scrollTo('how-it-works')}
                className="inline-flex items-center gap-2 px-8 py-4 bg-raised/50 hover:bg-raised text-ink-strong rounded-lg transition-all font-medium text-lg border border-strong"
              >
                Learn More
              </button>
            </div>

            <motion.p
              initial={{ opacity: 0 }}
              animate={isInView ? { opacity: 1 } : { opacity: 0 }}
              transition={{ duration: 1, delay: 0.6 }}
              className="mt-8 text-sm text-ink-muted"
            >
              Join entrepreneurs and investors who make better decisions with FEASIX
            </motion.p>
          </div>
        </motion.div>
      </div>
    </section>
  );
}
