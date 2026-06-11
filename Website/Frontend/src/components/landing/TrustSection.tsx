'use client';

import { useRef } from 'react';
import { motion, useInView } from 'framer-motion';
import { Shield, Lock, Database, FileCheck } from 'lucide-react';
import type { LucideIcon } from 'lucide-react';

interface TrustPrinciple {
  icon: LucideIcon;
  title: string;
  description: string;
}

const trustPrinciples: TrustPrinciple[] = [
  {
    icon: Shield,
    title: 'Privacy-First Architecture',
    description:
      'FEASIX is designed from the ground up to respect individual privacy. Our edge devices process visual data locally and transmit only aggregated, anonymized metrics—never raw images or video.',
  },
  {
    icon: Lock,
    title: 'No Raw Video Storage',
    description:
      'We do not store, transmit, or retain any raw video footage. Our hardware performs on-device processing to extract foot traffic counts and patterns, then discards source material immediately.',
  },
  {
    icon: Database,
    title: 'Aggregated Data Only',
    description:
      'All data transmitted to our analytics platform is fully aggregated and anonymized. Individual identities cannot be reconstructed from the data we collect or analyze.',
  },
  {
    icon: FileCheck,
    title: 'Transparent Methodology',
    description:
      'Our feasibility assessments are built on rigorous, auditable methods. We document our data sources, processing techniques, and modeling assumptions to maintain trust and accountability.',
  },
];

export function TrustSection() {
  const ref = useRef<HTMLDivElement>(null);
  const isInView = useInView(ref, { once: true, amount: 0.2 });

  return (
    <section id="trust" className="py-32 bg-page">
      <div className="max-w-7xl mx-auto px-6">
        <motion.div
          ref={ref}
          initial={{ opacity: 0, y: 30 }}
          animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
          transition={{ duration: 0.8 }}
          className="text-center mb-16"
        >
          <h2 className="mb-4">Trust, Responsibility &amp; Privacy</h2>
          <p className="text-xl text-ink-muted max-w-3xl mx-auto">
            FEASIX operates with the highest standards of privacy protection and data
            responsibility. Our technology delivers actionable intelligence without
            compromising individual privacy.
          </p>
        </motion.div>

        <div className="grid md:grid-cols-2 gap-8">
          {trustPrinciples.map((principle, index) => (
            <motion.div
              key={principle.title}
              initial={{ opacity: 0, y: 30 }}
              animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
              transition={{ duration: 0.6, delay: index * 0.15 }}
              className="bg-surface/30 border border-raised/50 rounded-xl p-8"
            >
              <div className="flex items-start gap-4">
                <div className="shrink-0 w-12 h-12 rounded-lg bg-raised flex items-center justify-center text-ink-muted">
                  <principle.icon className="w-6 h-6" />
                </div>
                <div>
                  <h3 className="mb-3">{principle.title}</h3>
                  <p className="text-ink-muted leading-relaxed">{principle.description}</p>
                </div>
              </div>
            </motion.div>
          ))}
        </div>

        <motion.div
          initial={{ opacity: 0, y: 30 }}
          animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
          transition={{ duration: 0.8, delay: 0.8 }}
          className="mt-16 bg-surface border border-raised rounded-xl p-8 text-center"
        >
          <p className="text-ink-body leading-relaxed max-w-4xl mx-auto">
            <span className="text-ink-strong font-medium">
              FEASIX is built for business intelligence, not surveillance.
            </span>{' '}
            We collect only what is necessary to assess location feasibility—foot traffic
            volumes, temporal patterns, and competitive context—and we do so in a manner
            that preserves individual anonymity. Privacy is not an afterthought; it is
            foundational to our platform architecture.
          </p>
        </motion.div>
      </div>
    </section>
  );
}
