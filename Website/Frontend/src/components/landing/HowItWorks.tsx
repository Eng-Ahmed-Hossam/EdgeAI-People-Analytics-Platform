'use client';

import { useRef } from 'react';
import { motion, useInView } from 'framer-motion';
import { MapPin, Cpu, BarChart3, CheckCircle2 } from 'lucide-react';
import type { LucideIcon } from 'lucide-react';

interface Step {
  icon: LucideIcon;
  title: string;
  description: string;
}

const steps: Step[] = [
  { icon: MapPin,        title: 'Real Location',        description: 'Select a business location and define your concept' },
  { icon: Cpu,           title: 'Edge Hardware',         description: 'Our sensors capture foot traffic and activity patterns' },
  { icon: BarChart3,     title: 'Decision Intelligence', description: 'The platform weighs evidence, confidence, risk, and financial assumptions' },
  { icon: CheckCircle2,  title: 'Feasibility Decision',  description: 'Receive your comprehensive feasibility assessment' },
];

export function HowItWorks() {
  const ref = useRef<HTMLDivElement>(null);
  const isInView = useInView(ref, { once: true, amount: 0.2 });

  return (
    <section id="how-it-works" className="py-32 bg-linear-to-b from-page to-surface">
      <div className="max-w-7xl mx-auto px-6">
        <motion.div
          ref={ref}
          initial={{ opacity: 0, y: 30 }}
          animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
          transition={{ duration: 0.8 }}
          className="text-center mb-20"
        >
          <h2 className="mb-4">How FEASIX Works</h2>
          <p className="text-xl text-ink-muted max-w-3xl mx-auto">
            From physical sensing to actionable intelligence: FEASIX delivers feasibility
            assessments grounded in real-world activity.
          </p>
        </motion.div>

        <div className="relative">
          {/* Connection line: accent → accent-data → accent gradient */}
          <div className="hidden lg:block absolute top-1/2 left-0 right-0 h-0.5 bg-linear-to-r from-accent via-accent-data to-accent -translate-y-1/2" />

          {/* Flow dots */}
          {isInView && (
            <>
              <motion.div
                className="hidden lg:block absolute top-1/2 left-0 w-3 h-3 bg-accent-data rounded-full -translate-y-1/2"
                animate={{ x: ['0%', '2400%'], opacity: [1, 1, 0] }}
                transition={{ duration: 4, repeat: Infinity, ease: 'linear', repeatDelay: 1 }}
              />
              <motion.div
                className="hidden lg:block absolute top-1/2 left-0 w-3 h-3 bg-accent-muted rounded-full -translate-y-1/2"
                animate={{ x: ['0%', '2400%'], opacity: [1, 1, 0] }}
                transition={{ duration: 4, repeat: Infinity, ease: 'linear', delay: 1, repeatDelay: 1 }}
              />
            </>
          )}

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8 relative z-10">
            {steps.map((step, index) => (
              <motion.div
                key={step.title}
                initial={{ opacity: 0, y: 40 }}
                animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 40 }}
                transition={{ duration: 0.6, delay: index * 0.2 }}
                className="relative"
              >
                <div className="bg-surface border border-raised rounded-xl p-8 text-center">
                  <div className="mb-6 flex justify-center">
                    <div className="relative">
                      <div className="w-16 h-16 rounded-full bg-accent/20 flex items-center justify-center">
                        <step.icon className="w-8 h-8 text-accent-muted" />
                      </div>
                      <motion.div
                        className="absolute inset-0 rounded-full border-2 border-accent-muted"
                        animate={{ scale: [1, 1.3, 1.3], opacity: [0.5, 0, 0] }}
                        transition={{ duration: 2, repeat: Infinity, delay: index * 0.3 }}
                      />
                    </div>
                  </div>
                  <div className="text-sm text-ink-faint mb-2">Step {index + 1}</div>
                  <h3 className="mb-3">{step.title}</h3>
                  <p className="text-ink-muted text-sm leading-relaxed">{step.description}</p>
                </div>

                {index < steps.length - 1 && (
                  <motion.div
                    className="hidden lg:block absolute top-1/2 -right-4 -translate-y-1/2 z-20"
                    animate={{ x: [0, 5, 0] }}
                    transition={{ duration: 2, repeat: Infinity, delay: index * 0.2 }}
                  >
                    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" className="text-accent-muted">
                      <path
                        d="M5 12h14m-6-6 6 6-6 6"
                        stroke="currentColor"
                        strokeWidth="2"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                      />
                    </svg>
                  </motion.div>
                )}
              </motion.div>
            ))}
          </div>
        </div>

        <motion.div
          initial={{ opacity: 0 }}
          animate={isInView ? { opacity: 1 } : { opacity: 0 }}
          transition={{ duration: 1, delay: 1 }}
          className="mt-16 text-center"
        >
          <div className="inline-flex items-center gap-2 px-6 py-3 bg-surface border border-raised rounded-lg text-ink-body">
            <Cpu className="w-4 h-4 text-accent-data" />
            <span className="text-sm">Powered by hardware-assisted evidence collection at physical locations</span>
          </div>
        </motion.div>
      </div>
    </section>
  );
}
