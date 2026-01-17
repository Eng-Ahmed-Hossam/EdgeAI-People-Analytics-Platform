'use client';

import { motion, AnimatePresence, useInView } from 'framer-motion';
import { MapPin, Circle } from 'lucide-react';
import { useRef, useState } from 'react';
import Image from 'next/image';

const locations = [
    { id: 1, x: 25, y: 30, score: 87, name: 'Downtown Core', status: 'high' },
    { id: 2, x: 45, y: 45, score: 72, name: 'Tech District', status: 'medium' },
    { id: 3, x: 65, y: 35, score: 91, name: 'Financial Hub', status: 'high' },
    { id: 4, x: 35, y: 60, score: 64, name: 'Suburban Plaza', status: 'medium' },
    { id: 5, x: 75, y: 55, score: 58, name: 'Retail Strip', status: 'low' },
    { id: 6, x: 55, y: 25, score: 79, name: 'University Area', status: 'medium' },
];

export function MapPreview() {
    const ref = useRef<HTMLDivElement | null>(null);
    const isInView = useInView(ref, { once: true, amount: 0.2 });
    const [hoveredLocation, setHoveredLocation] = useState<number | null>(null);

    const getStatusColor = (status: string) => {
        switch (status) {
            case 'high':
                return 'bg-green-500 border-green-400';
            case 'medium':
                return 'bg-cyan-500 border-cyan-400';
            case 'low':
                return 'bg-orange-500 border-orange-400';
            default:
                return 'bg-slate-500 border-slate-400';
        }
    };

    return (
        <section
            id="map-preview"
            className="py-32 bg-gradient-to-b from-slate-900 to-slate-950"
        >
            <div className="max-w-7xl mx-auto px-6">
                {/* Header */}
                <motion.div
                    ref={ref}
                    initial={{ opacity: 0, y: 30 }}
                    animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
                    transition={{ duration: 0.8 }}
                    className="text-center mb-16"
                >
                    <h2 className="mb-4 text-white">Explore Feasibility Anywhere</h2>
                    <p className="text-xl text-slate-400 max-w-3xl mx-auto">
                        A conceptual preview of how FEASIX evaluates business feasibility across
                        locations using spatial intelligence.
                    </p>
                </motion.div>

                {/* Map Card */}
                <motion.div
                    initial={{ opacity: 0, y: 40 }}
                    animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 40 }}
                    transition={{ duration: 0.8, delay: 0.3 }}
                    className="relative bg-slate-900 border border-slate-800 rounded-2xl p-8 overflow-hidden"
                >
                    {/* Base Map Image (Google/Mapbox-like feel) */}
                    <div className="absolute inset-0 opacity-20">
                        <Image
                            src="/map-base.png"
                            alt=""
                            fill
                            className="object-cover"
                        />
                    </div>

                    {/* Grid Overlay */}
                    <div className="absolute inset-0 opacity-5">
                        <div
                            className="w-full h-full"
                            style={{
                                backgroundImage:
                                    'linear-gradient(to right, #3b82f6 1px, transparent 1px), linear-gradient(to bottom, #3b82f6 1px, transparent 1px)',
                                backgroundSize: '40px 40px',
                            }}
                        />
                    </div>

                    {/* Legend */}
                    <div className="relative z-10 mb-6 flex items-center justify-between">
                        <h3 className="text-white">Metropolitan Feasibility Overview</h3>
                        <div className="flex items-center gap-6">
                            <Legend color="green" label="High" />
                            <Legend color="cyan" label="Medium" />
                            <Legend color="orange" label="Lower" />
                        </div>
                    </div>

                    {/* Map Area */}
                    <div className="relative z-10 h-[500px] bg-slate-950/80 rounded-xl border border-slate-800 overflow-hidden">
                        {/* Glow */}
                        <motion.div
                            className="absolute inset-0 opacity-20"
                            animate={{
                                background: [
                                    'radial-gradient(circle at 30% 40%, #3b82f6 0%, transparent 50%)',
                                    'radial-gradient(circle at 70% 60%, #06b6d4 0%, transparent 50%)',
                                    'radial-gradient(circle at 30% 40%, #3b82f6 0%, transparent 50%)',
                                ],
                            }}
                            transition={{ duration: 10, repeat: Infinity, ease: 'linear' }}
                        />

                        {/* Pins */}
                        {locations.map((location, index) => (
                            <motion.div
                                key={location.id}
                                className="absolute"
                                style={{
                                    left: `${location.x}%`,
                                    top: `${location.y}%`,
                                    transform: 'translate(-50%, -50%)',
                                }}
                                initial={{ opacity: 0, scale: 0 }}
                                animate={isInView ? { opacity: 1, scale: 1 } : {}}
                                transition={{ duration: 0.6, delay: 0.5 + index * 0.1 }}
                                onMouseEnter={() => setHoveredLocation(location.id)}
                                onMouseLeave={() => setHoveredLocation(null)}
                            >
                                {/* Pulse */}
                                <motion.div
                                    className={`absolute inset-0 rounded-full ${getStatusColor(location.status)} blur-xl`}
                                    animate={{ scale: [1, 2, 1], opacity: [0.5, 0, 0.5] }}
                                    transition={{ duration: 3, repeat: Infinity }}
                                />

                                {/* Dot */}
                                <div
                                    className={`relative w-4 h-4 rounded-full ${getStatusColor(location.status)} ${hoveredLocation === location.id ? 'scale-150' : ''
                                        }`}
                                />

                                {/* Tooltip */}
                                <AnimatePresence>
                                    {hoveredLocation === location.id && (
                                        <motion.div
                                            initial={{ opacity: 0, y: 10 }}
                                            animate={{ opacity: 1, y: 0 }}
                                            exit={{ opacity: 0, y: 10 }}
                                            className="absolute top-full left-1/2 -translate-x-1/2 mt-4 bg-slate-900 border border-slate-700 rounded-lg p-4 shadow-xl z-20"
                                        >
                                            <div className="flex items-center gap-2 mb-1">
                                                <MapPin className="w-4 h-4 text-cyan-400" />
                                                <span className="text-white font-medium">{location.name}</span>
                                            </div>
                                            <div className="text-sm text-slate-400">Feasibility</div>
                                            <div className="text-2xl font-semibold text-white">
                                                {location.score}/100
                                            </div>
                                        </motion.div>
                                    )}
                                </AnimatePresence>
                            </motion.div>
                        ))}
                    </div>

                    {/* Footer */}
                    <div className="relative z-10 mt-6 flex items-center justify-between">
                        <span className="text-xs text-slate-500">
                            Conceptual preview — full interactive map available in product
                        </span>
                        <button className="px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white rounded-lg text-sm font-medium">
                            Explore Full Map
                        </button>
                    </div>
                </motion.div>
            </div>
        </section>
    );
}

function Legend({ color, label }: { color: string; label: string }) {
    return (
        <div className="flex items-center gap-2 text-sm text-slate-400">
            <Circle className={`w-3 h-3 fill-${color}-500 text-${color}-500`} />
            {label}
        </div>
    );
}
