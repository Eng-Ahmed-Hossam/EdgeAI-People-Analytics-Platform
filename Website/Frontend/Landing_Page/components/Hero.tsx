'use client';

import { motion } from 'framer-motion';
import Image from 'next/image';

export function Hero() {
    const scrollToSection = (sectionId: string) => {
        const element = document.getElementById(sectionId);
        if (element) {
            element.scrollIntoView({ behavior: 'smooth', block: 'start' });
        }
    };

    return (
        <section className="relative min-h-screen flex items-center justify-center overflow-hidden">
            {/* Background Stack */}
            <div className="absolute inset-0">
                {/* Animated Image */}
                <motion.div
                    className="absolute inset-0 opacity-40"
                    animate={{ scale: [1, 1.05, 1] }}
                    transition={{
                        duration: 20,
                        repeat: Infinity,
                        ease: 'easeInOut',
                    }}
                >
                    <Image
                        src="/hero-bg.png"
                        alt=""
                        fill
                        priority
                        className="object-cover"
                    />
                </motion.div>

                {/* Dark Overlay */}
                <div className="absolute inset-0 bg-slate-950/60" />

                {/* Gradient Overlay */}
                <div className="absolute inset-0 bg-gradient-to-b from-slate-950/40 via-slate-950/75 to-slate-950" />

                {/* Subtle Data Dots */}
                {Array.from({ length: 15 }).map((_, i) => (
                    <motion.div
                        key={i}
                        className="absolute w-1 h-1 bg-cyan-400 rounded-full"
                        style={{
                            left: `${(i * 17) % 100}%`,
                            top: `${(i * 29) % 100}%`,
                        }}
                        animate={{
                            opacity: [0.2, 0.8, 0.2],
                            scale: [1, 1.5, 1],
                        }}
                        transition={{
                            duration: 5,
                            repeat: Infinity,
                            delay: i * 0.2,
                        }}
                    />
                ))}
            </div>

            {/* Content */}
            <div className="relative z-10 max-w-5xl mx-auto px-6 text-center">
                <motion.h1
                    className="text-6xl font-semibold mb-6 text-white"
                    initial={{ opacity: 0, y: 30 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.8 }}
                >
                    Make Business Decisions With Confidence —{' '}
                    <span className="text-blue-400">Before You Invest</span>
                </motion.h1>

                <motion.p
                    className="text-xl text-slate-300 mb-12 max-w-3xl mx-auto leading-relaxed"
                    initial={{ opacity: 0, y: 30 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.8, delay: 0.2 }}
                >
                    FEASIX combines real-world location activity, hardware sensing, and
                    financial modeling to deliver objective business feasibility
                    assessments.
                </motion.p>

                <motion.div
                    className="flex items-center justify-center gap-4"
                    initial={{ opacity: 0, y: 30 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.8, delay: 0.4 }}
                >
                    <button
                        onClick={() => scrollToSection('map-preview')}
                        className="px-8 py-4 bg-blue-600 hover:bg-blue-700 text-white rounded-lg transition-all font-medium text-lg shadow-lg shadow-blue-900/50"
                    >
                        Explore Feasibility on the Map
                    </button>
                    <button
                        onClick={() => scrollToSection('how-it-works')}
                        className="px-8 py-4 bg-slate-800/50 hover:bg-slate-800 text-white rounded-lg transition-all font-medium text-lg border border-slate-700"
                    >
                        See How FEASIX Works
                    </button>
                </motion.div>

                {/* Indicators */}
                <motion.div
                    className="mt-20 flex items-center justify-center gap-16 text-sm text-slate-400"
                    initial={{ opacity: 0 }}
                    animate={{ opacity: 1 }}
                    transition={{ duration: 1, delay: 1 }}
                >
                    <Indicator label="Real-Time Data" color="bg-cyan-400" />
                    <Indicator label="Edge Hardware" color="bg-blue-400" />
                    <Indicator label="AI Analytics" color="bg-indigo-400" />
                </motion.div>
            </div>

            {/* Bottom Fade */}
            <div className="absolute bottom-0 left-0 right-0 h-32 bg-gradient-to-t from-slate-950 to-transparent" />
        </section>
    );
}

function Indicator({
    label,
    color,
}: {
    label: string;
    color: string;
}) {
    return (
        <div className="flex items-center gap-2">
            <div className={`w-2 h-2 rounded-full animate-pulse ${color}`} />
            <span>{label}</span>
        </div>
    );
}
