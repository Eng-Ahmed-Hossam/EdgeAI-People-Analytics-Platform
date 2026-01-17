'use client';

import { motion, useInView } from 'framer-motion';
import { Shield, Lock, Database, FileCheck } from 'lucide-react';
import { useRef } from 'react';

const trustPrinciples = [
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
    const ref = useRef<HTMLDivElement | null>(null);
    const isInView = useInView(ref, { once: true, amount: 0.2 });

    return (
        <section id="trust" className="py-32 bg-slate-950">
            <div className="max-w-7xl mx-auto px-6">
                {/* Header */}
                <motion.div
                    ref={ref}
                    initial={{ opacity: 0, y: 30 }}
                    animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
                    transition={{ duration: 0.8 }}
                    className="text-center mb-16"
                >
                    <h2 className="mb-4 text-white">
                        Trust, Responsibility & Privacy
                    </h2>
                    <p className="text-xl text-slate-400 max-w-3xl mx-auto">
                        FEASIX operates with the highest standards of privacy protection and data
                        responsibility. Our technology delivers actionable intelligence without
                        compromising individual privacy.
                    </p>
                </motion.div>

                {/* Principles */}
                <div className="grid md:grid-cols-2 gap-8">
                    {trustPrinciples.map((principle, index) => (
                        <motion.div
                            key={principle.title}
                            initial={{ opacity: 0, y: 30 }}
                            animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
                            transition={{ duration: 0.6, delay: index * 0.15 }}
                            className="bg-slate-900/30 border border-slate-800/50 rounded-xl p-8"
                        >
                            <div className="flex items-start gap-4">
                                <div className="flex-shrink-0 w-12 h-12 rounded-lg bg-slate-800 flex items-center justify-center text-slate-400">
                                    <principle.icon className="w-6 h-6" />
                                </div>
                                <div>
                                    <h3 className="text-white mb-3">
                                        {principle.title}
                                    </h3>
                                    <p className="text-slate-400 leading-relaxed">
                                        {principle.description}
                                    </p>
                                </div>
                            </div>
                        </motion.div>
                    ))}
                </div>

                {/* Closing Statement */}
                <motion.div
                    initial={{ opacity: 0, y: 30 }}
                    animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
                    transition={{ duration: 0.8, delay: 0.8 }}
                    className="mt-16 bg-slate-900 border border-slate-800 rounded-xl p-8 text-center"
                >
                    <p className="text-slate-300 leading-relaxed max-w-4xl mx-auto">
                        <span className="text-white font-medium">
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
