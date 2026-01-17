'use client';

import { motion, useInView } from 'framer-motion';
import { ArrowRight, Map } from 'lucide-react';
import { useRef } from 'react';

export function FinalCTA() {
    const ref = useRef(null);
    const isInView = useInView(ref, { once: true, amount: 0.3 });

    const scrollToSection = (sectionId: string) => {
        const element = document.getElementById(sectionId);
        if (element) {
            element.scrollIntoView({ behavior: 'smooth', block: 'start' });
        }
    };

    return (
        <section className="py-32 bg-gradient-to-b from-slate-900 to-slate-950">
            <div className="max-w-7xl mx-auto px-6">
                <motion.div
                    ref={ref}
                    initial={{ opacity: 0, y: 40 }}
                    animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 40 }}
                    transition={{ duration: 0.8 }}
                    className="relative bg-gradient-to-br from-blue-950 to-slate-900 border border-blue-900/50 rounded-2xl p-16 text-center overflow-hidden"
                >
                    {/* Background Pattern */}
                    <div className="absolute inset-0 opacity-10">
                        <div
                            className="w-full h-full"
                            style={{
                                backgroundImage:
                                    'linear-gradient(to right, #3b82f6 1px, transparent 1px), linear-gradient(to bottom, #3b82f6 1px, transparent 1px)',
                                backgroundSize: '60px 60px',
                            }}
                        />
                    </div>

                    {/* Animated Glow */}
                    <motion.div
                        className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-96 h-96 bg-blue-600 rounded-full blur-3xl opacity-20"
                        animate={{
                            scale: [1, 1.2, 1],
                            opacity: [0.2, 0.3, 0.2],
                        }}
                        transition={{
                            duration: 4,
                            repeat: Infinity,
                            ease: 'easeInOut',
                        }}
                    />

                    <div className="relative z-10">
                        <motion.div
                            initial={{ opacity: 0, scale: 0.9 }}
                            animate={isInView ? { opacity: 1, scale: 1 } : { opacity: 0, scale: 0.9 }}
                            transition={{ duration: 0.8, delay: 0.2 }}
                            className="inline-flex items-center justify-center w-16 h-16 rounded-full bg-blue-600/20 border border-blue-500/30 mb-8"
                        >
                            <Map className="w-8 h-8 text-blue-400" />
                        </motion.div>

                        <h2 className="mb-6 text-white max-w-3xl mx-auto">
                            Ready to Make Data-Backed Business Decisions?
                        </h2>

                        <p className="text-xl text-slate-300 mb-10 max-w-2xl mx-auto leading-relaxed">
                            Explore FEASIX's interactive map and discover feasibility intelligence for
                            locations across your target market. Start with confidence.
                        </p>

                        <div className="flex items-center justify-center gap-4">
                            <motion.button
                                onClick={() => scrollToSection('map-preview')}
                                whileHover={{ scale: 1.05 }}
                                whileTap={{ scale: 0.95 }}
                                className="inline-flex items-center gap-2 px-8 py-4 bg-blue-600 hover:bg-blue-700 text-white rounded-lg transition-all font-medium text-lg shadow-lg shadow-blue-900/50"
                            >
                                Explore the Map
                                <ArrowRight className="w-5 h-5" />
                            </motion.button>

                            <motion.button
                                onClick={() => window.scrollTo({ top: 0, behavior: 'smooth' })}
                                whileHover={{ scale: 1.05 }}
                                whileTap={{ scale: 0.95 }}
                                className="inline-flex items-center gap-2 px-8 py-4 bg-slate-800/50 hover:bg-slate-800 text-white rounded-lg transition-all font-medium text-lg border border-slate-700"
                            >
                                Learn More
                            </motion.button>
                        </div>

                        <motion.p
                            initial={{ opacity: 0 }}
                            animate={isInView ? { opacity: 1 } : { opacity: 0 }}
                            transition={{ duration: 1, delay: 0.6 }}
                            className="mt-8 text-sm text-slate-400"
                        >
                            Join entrepreneurs and investors who make better decisions with FEASIX
                        </motion.p>
                    </div>
                </motion.div>
            </div>
        </section>
    );
}