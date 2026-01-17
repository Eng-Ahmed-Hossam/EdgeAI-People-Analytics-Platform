'use client';

import { motion, useInView } from 'framer-motion';
import { Target, MapPin, TrendingUp, Shield } from 'lucide-react';
import { useRef } from 'react';

const features = [
    {
        icon: Target,
        title: 'Business Feasibility Testing',
        description:
            'Validate your business concept with real-world data before committing capital. Our platform analyzes location dynamics, market conditions, and financial projections to provide clear feasibility scores.',
    },
    {
        icon: MapPin,
        title: 'Location-Driven Insight',
        description:
            'Every location tells a story. FEASIX captures foot traffic patterns, demographic flows, and competitive density to help you understand the true potential of any business address.',
    },
    {
        icon: TrendingUp,
        title: 'Financial Risk Estimation',
        description:
            'Move beyond guesswork. Our financial modeling engine combines location data with industry benchmarks to project revenue ranges, break-even timelines, and risk-adjusted returns.',
    },
    {
        icon: Shield,
        title: 'Confidence Before Investment',
        description:
            'Make investor-grade decisions with data-backed confidence. FEASIX provides the analytical foundation you need to pursue opportunities—or avoid costly mistakes.',
    },
];

function FeatureCard({
    feature,
    index,
}: {
    feature: (typeof features)[0];
    index: number;
}) {
    const ref = useRef<HTMLDivElement | null>(null);
    const isInView = useInView(ref, { once: true, amount: 0.3 });

    return (
        <motion.div
            ref={ref}
            initial={{ opacity: 0, y: 40 }}
            animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 40 }}
            transition={{ duration: 0.6, delay: index * 0.1 }}
            className="group relative bg-slate-900/50 border border-slate-800 rounded-xl p-8 hover:border-blue-800/50 hover:bg-slate-900/80 transition-all duration-300"
        >
            <div className="mb-5">
                <div className="inline-flex items-center justify-center w-12 h-12 rounded-lg bg-blue-600/10 text-blue-400 group-hover:bg-blue-600/20 transition-colors">
                    <feature.icon className="w-6 h-6" />
                </div>
            </div>

            <h3 className="mb-3 text-white">{feature.title}</h3>
            <p className="text-slate-400 leading-relaxed">
                {feature.description}
            </p>
        </motion.div>
    );
}

export function ProductValue() {
    const ref = useRef<HTMLDivElement | null>(null);
    const isInView = useInView(ref, { once: true, amount: 0.2 });

    return (
        <section id="product-value" className="py-32 bg-slate-950">
            <div className="max-w-7xl mx-auto px-6">
                <motion.div
                    ref={ref}
                    initial={{ opacity: 0, y: 30 }}
                    animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
                    transition={{ duration: 0.8 }}
                    className="text-center mb-16"
                >
                    <h2 className="mb-4 text-white">
                        Objective Feasibility Intelligence
                    </h2>
                    <p className="text-xl text-slate-400 max-w-3xl mx-auto">
                        FEASIX transforms how entrepreneurs and investors evaluate business
                        opportunities—replacing intuition with data-backed certainty.
                    </p>
                </motion.div>

                <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                    {features.map((feature, index) => (
                        <FeatureCard
                            key={feature.title}
                            feature={feature}
                            index={index}
                        />
                    ))}
                </div>
            </div>
        </section>
    );
}
