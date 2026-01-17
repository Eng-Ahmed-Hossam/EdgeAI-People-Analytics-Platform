'use client';

import { motion, useInView } from 'framer-motion';
import { MapPin, Cpu, BarChart3, CheckCircle2 } from 'lucide-react';
import { useRef } from 'react';

const steps = [
    {
        icon: MapPin,
        title: 'Real Location',
        description: 'Select a business location and define your concept',
    },
    {
        icon: Cpu,
        title: 'Edge Hardware',
        description: 'Our sensors capture foot traffic and activity patterns',
    },
    {
        icon: BarChart3,
        title: 'Analytics Engine',
        description: 'AI processes location data and financial models',
    },
    {
        icon: CheckCircle2,
        title: 'Feasibility Decision',
        description: 'Receive your comprehensive feasibility assessment',
    },
];

export function HowItWorks() {
    const ref = useRef<HTMLDivElement | null>(null);
    const isInView = useInView(ref, { once: true, amount: 0.2 });

    return (
        <section
            id="how-it-works"
            className="py-32 bg-gradient-to-b from-slate-950 to-slate-900"
        >
            <div className="max-w-7xl mx-auto px-6">
                {/* Header */}
                <motion.div
                    ref={ref}
                    initial={{ opacity: 0, y: 30 }}
                    animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
                    transition={{ duration: 0.8 }}
                    className="text-center mb-20"
                >
                    <h2 className="mb-4 text-white">How FEASIX Works</h2>
                    <p className="text-xl text-slate-400 max-w-3xl mx-auto">
                        From physical sensing to actionable intelligence—FEASIX delivers
                        feasibility assessments grounded in real-world activity.
                    </p>
                </motion.div>

                <div className="relative">
                    {/* Connection Line */}
                    <div className="hidden lg:block absolute top-1/2 left-0 right-0 h-0.5 bg-gradient-to-r from-blue-600 via-cyan-500 to-blue-600 -translate-y-1/2" />

                    {/* Flow Dots */}
                    {isInView && (
                        <>
                            <motion.div
                                className="hidden lg:block absolute top-1/2 left-0 w-3 h-3 bg-cyan-400 rounded-full -translate-y-1/2"
                                animate={{ x: ['0%', '2400%'], opacity: [1, 1, 0] }}
                                transition={{
                                    duration: 4,
                                    repeat: Infinity,
                                    ease: 'linear',
                                    repeatDelay: 1,
                                }}
                            />
                            <motion.div
                                className="hidden lg:block absolute top-1/2 left-0 w-3 h-3 bg-blue-400 rounded-full -translate-y-1/2"
                                animate={{ x: ['0%', '2400%'], opacity: [1, 1, 0] }}
                                transition={{
                                    duration: 4,
                                    repeat: Infinity,
                                    ease: 'linear',
                                    delay: 1,
                                    repeatDelay: 1,
                                }}
                            />
                        </>
                    )}

                    {/* Steps */}
                    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8 relative z-10">
                        {steps.map((step, index) => (
                            <motion.div
                                key={step.title}
                                initial={{ opacity: 0, y: 40 }}
                                animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 40 }}
                                transition={{ duration: 0.6, delay: index * 0.2 }}
                                className="relative"
                            >
                                <div className="bg-slate-900 border border-slate-800 rounded-xl p-8 text-center">
                                    <div className="mb-6 flex justify-center">
                                        <div className="relative">
                                            <div className="w-16 h-16 rounded-full bg-blue-600/20 flex items-center justify-center">
                                                <step.icon className="w-8 h-8 text-blue-400" />
                                            </div>
                                            <motion.div
                                                className="absolute inset-0 rounded-full border-2 border-blue-400"
                                                animate={{
                                                    scale: [1, 1.3, 1.3],
                                                    opacity: [0.5, 0, 0],
                                                }}
                                                transition={{
                                                    duration: 2,
                                                    repeat: Infinity,
                                                    delay: index * 0.3,
                                                }}
                                            />
                                        </div>
                                    </div>

                                    <div className="text-sm text-slate-500 mb-2">
                                        Step {index + 1}
                                    </div>
                                    <h3 className="mb-3 text-white">{step.title}</h3>
                                    <p className="text-slate-400 text-sm leading-relaxed">
                                        {step.description}
                                    </p>
                                </div>

                                {index < steps.length - 1 && (
                                    <motion.div
                                        className="hidden lg:block absolute top-1/2 -right-4 -translate-y-1/2 z-20"
                                        animate={{ x: [0, 5, 0] }}
                                        transition={{
                                            duration: 2,
                                            repeat: Infinity,
                                            delay: index * 0.2,
                                        }}
                                    >
                                        <svg
                                            width="24"
                                            height="24"
                                            viewBox="0 0 24 24"
                                            fill="none"
                                            className="text-blue-500"
                                        >
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

                {/* Footer Note */}
                <motion.div
                    initial={{ opacity: 0 }}
                    animate={isInView ? { opacity: 1 } : { opacity: 0 }}
                    transition={{ duration: 1, delay: 1 }}
                    className="mt-16 text-center"
                >
                    <div className="inline-flex items-center gap-2 px-6 py-3 bg-slate-900 border border-slate-800 rounded-lg text-slate-300">
                        <Cpu className="w-4 h-4 text-cyan-400" />
                        <span className="text-sm">
                            Powered by hardware-assisted data collection at physical locations
                        </span>
                    </div>
                </motion.div>
            </div>
        </section>
    );
}
