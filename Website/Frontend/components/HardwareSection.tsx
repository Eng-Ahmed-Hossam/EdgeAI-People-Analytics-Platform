'use client';

import { motion, useInView } from 'framer-motion';
import { Activity, Camera, Wifi, Database } from 'lucide-react';
import { useRef } from 'react';

const hardwareFeatures = [
    {
        icon: Activity,
        title: 'Real-Time Sensing',
        description: 'Continuously monitor foot traffic and activity levels',
    },
    {
        icon: Camera,
        title: 'Privacy-First Design',
        description: 'No raw video storage—only aggregated, anonymous metrics',
    },
    {
        icon: Wifi,
        title: 'Edge Processing',
        description: 'Data processed locally before transmission',
    },
    {
        icon: Database,
        title: 'Secure Pipeline',
        description: 'Encrypted transmission to analytics infrastructure',
    },
];

export function HardwareSection() {
    const ref = useRef<HTMLDivElement | null>(null);
    const isInView = useInView(ref, { once: true, amount: 0.2 });

    return (
        <section id="hardware" className="py-32 bg-slate-900">
            <div className="max-w-7xl mx-auto px-6">
                <div className="grid lg:grid-cols-2 gap-16 items-center">

                    {/* Left: Visual Representation */}
                    <motion.div
                        ref={ref}
                        initial={{ opacity: 0, x: -40 }}
                        animate={isInView ? { opacity: 1, x: 0 } : { opacity: 0, x: -40 }}
                        transition={{ duration: 0.8 }}
                        className="relative"
                    >
                        <div className="relative bg-slate-950 border border-slate-800 rounded-2xl p-12 overflow-hidden">

                            {/* Central Device */}
                            <div className="relative z-10">
                                <div className="w-32 h-32 mx-auto bg-gradient-to-br from-blue-600 to-cyan-600 rounded-2xl shadow-2xl shadow-blue-900/50 flex items-center justify-center">
                                    <Activity className="w-16 h-16 text-white" />
                                </div>

                                <div className="mt-8 text-center">
                                    <h3 className="text-white mb-2">FEASIX Edge Device</h3>
                                    <p className="text-sm text-slate-500">
                                        Deployed at Target Location
                                    </p>
                                </div>
                            </div>

                            {/* Connection Pulses */}
                            {[0, 1, 2, 3].map((i) => (
                                <motion.div
                                    key={i}
                                    className="absolute w-2 h-2 bg-cyan-400 rounded-full"
                                    style={{ top: '50%', left: '50%' }}
                                    animate={{
                                        x: [0, Math.cos(i * Math.PI / 2) * 100, 0],
                                        y: [0, Math.sin(i * Math.PI / 2) * 100, 0],
                                        opacity: [0, 1, 0],
                                        scale: [0, 1.5, 0],
                                    }}
                                    transition={{
                                        duration: 3,
                                        repeat: Infinity,
                                        delay: i * 0.5,
                                        ease: 'easeInOut',
                                    }}
                                />
                            ))}

                            {/* Background Grid */}
                            <div className="absolute inset-0 opacity-10">
                                <div
                                    className="w-full h-full"
                                    style={{
                                        backgroundImage:
                                            'linear-gradient(to right, #3b82f6 1px, transparent 1px), linear-gradient(to bottom, #3b82f6 1px, transparent 1px)',
                                        backgroundSize: '40px 40px',
                                    }}
                                />
                            </div>
                        </div>
                    </motion.div>

                    {/* Right: Content */}
                    <motion.div
                        initial={{ opacity: 0, x: 40 }}
                        animate={isInView ? { opacity: 1, x: 0 } : { opacity: 0, x: 40 }}
                        transition={{ duration: 0.8, delay: 0.2 }}
                    >
                        <h2 className="mb-4 text-white">
                            Hardware-Assisted Feasibility Intelligence
                        </h2>

                        <p className="text-xl text-slate-400 mb-8">
                            FEASIX deploys purpose-built edge devices at physical locations to capture
                            real-world activity that directly impacts business feasibility.
                        </p>

                        <div className="space-y-6">
                            {hardwareFeatures.map((feature, index) => (
                                <motion.div
                                    key={feature.title}
                                    initial={{ opacity: 0, x: 20 }}
                                    animate={isInView ? { opacity: 1, x: 0 } : { opacity: 0, x: 20 }}
                                    transition={{ duration: 0.6, delay: 0.3 + index * 0.1 }}
                                    className="flex items-start gap-4"
                                >
                                    <div className="flex-shrink-0 w-10 h-10 rounded-lg bg-blue-600/10 flex items-center justify-center">
                                        <feature.icon className="w-5 h-5 text-blue-400" />
                                    </div>
                                    <div>
                                        <h4 className="text-white mb-1 font-medium">
                                            {feature.title}
                                        </h4>
                                        <p className="text-slate-400 text-sm">
                                            {feature.description}
                                        </p>
                                    </div>
                                </motion.div>
                            ))}
                        </div>

                        <motion.div
                            initial={{ opacity: 0 }}
                            animate={isInView ? { opacity: 1 } : { opacity: 0 }}
                            transition={{ duration: 1, delay: 0.8 }}
                            className="mt-8 p-4 bg-slate-950 border border-slate-800 rounded-lg"
                        >
                            <p className="text-sm text-slate-400">
                                <span className="text-cyan-400 font-medium">
                                    Physical sensing
                                </span>{' '}
                                provides ground truth that digital-only approaches cannot replicate—
                                real foot traffic, temporal patterns, and competitive dynamics.
                            </p>
                        </motion.div>
                    </motion.div>

                </div>
            </div>
        </section>
    );
}
