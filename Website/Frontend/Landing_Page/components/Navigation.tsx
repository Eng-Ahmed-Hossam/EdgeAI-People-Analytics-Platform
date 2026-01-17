'use client';

import { useState, useRef, useEffect } from 'react';
import { ChevronDown, Menu, X } from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';

interface DropdownItem {
    label: string;
    action: () => void;
}

interface NavItem {
    label: string;
    items: DropdownItem[];
}

export function Navigation() {
    const [activeDropdown, setActiveDropdown] = useState<string | null>(null);
    const [mobileOpen, setMobileOpen] = useState(false);
    const dropdownRef = useRef<HTMLDivElement | null>(null);

    const scrollToSection = (sectionId: string) => {
        const element = document.getElementById(sectionId);
        if (element) {
            element.scrollIntoView({ behavior: 'smooth', block: 'start' });
        }
        setActiveDropdown(null);
        setMobileOpen(false);
    };

    const navItems: NavItem[] = [
        {
            label: 'Product',
            items: [
                { label: 'Feasibility Engine', action: () => scrollToSection('product-value') },
                { label: 'Location Intelligence', action: () => scrollToSection('map-preview') },
                { label: 'Financial Modeling', action: () => scrollToSection('product-preview') },
            ],
        },
        {
            label: 'Technology',
            items: [
                { label: 'Edge Hardware', action: () => scrollToSection('hardware') },
                { label: 'Data Pipeline', action: () => scrollToSection('how-it-works') },
                { label: 'AI & Analytics', action: () => scrollToSection('product-preview') },
            ],
        },
        {
            label: 'Use Cases',
            items: [
                { label: 'Entrepreneurs', action: () => scrollToSection('product-value') },
                { label: 'Investors', action: () => scrollToSection('product-value') },
                { label: 'Urban Planning', action: () => scrollToSection('map-preview') },
            ],
        },
    ];

    useEffect(() => {
        function handleClickOutside(event: MouseEvent) {
            if (
                dropdownRef.current &&
                !dropdownRef.current.contains(event.target as Node)
            ) {
                setActiveDropdown(null);
            }
        }

        document.addEventListener('mousedown', handleClickOutside);
        return () =>
            document.removeEventListener('mousedown', handleClickOutside);
    }, []);

    return (
        <>
            {/* NAVBAR */}
            <nav className="fixed top-0 left-0 right-0 z-50 bg-slate-950/80 backdrop-blur-xl border-b border-slate-800/50">
                <div className="max-w-7xl mx-auto px-6 py-4">
                    <div className="flex items-center justify-between">
                        {/* Left */}
                        <div className="flex items-center gap-12">
                            <div className="text-2xl font-semibold tracking-tight text-white">
                                FEASIX
                            </div>

                            {/* Desktop Nav */}
                            <div
                                ref={dropdownRef}
                                className="hidden md:flex items-center gap-8"
                            >
                                {navItems.map((item) => (
                                    <div key={item.label} className="relative">
                                        <button
                                            onClick={() =>
                                                setActiveDropdown(
                                                    activeDropdown === item.label ? null : item.label
                                                )
                                            }
                                            aria-expanded={activeDropdown === item.label}
                                            className="flex items-center gap-1 text-slate-300 hover:text-white transition-colors text-sm"
                                        >
                                            {item.label}
                                            <ChevronDown className="w-4 h-4" />
                                        </button>

                                        <AnimatePresence>
                                            {activeDropdown === item.label && (
                                                <motion.div
                                                    initial={{ opacity: 0, y: -10 }}
                                                    animate={{ opacity: 1, y: 0 }}
                                                    exit={{ opacity: 0, y: -10 }}
                                                    transition={{ duration: 0.2 }}
                                                    className="absolute top-full left-0 mt-2 w-56 bg-slate-900 border border-slate-800 rounded-lg shadow-2xl overflow-hidden"
                                                >
                                                    {item.items.map((dropdownItem) => (
                                                        <button
                                                            key={dropdownItem.label}
                                                            onClick={dropdownItem.action}
                                                            className="w-full text-left px-4 py-3 text-sm text-slate-300 hover:bg-slate-800 hover:text-white transition-colors"
                                                        >
                                                            {dropdownItem.label}
                                                        </button>
                                                    ))}
                                                </motion.div>
                                            )}
                                        </AnimatePresence>
                                    </div>
                                ))}

                                <button
                                    onClick={() => scrollToSection('map-preview')}
                                    className="text-slate-300 hover:text-white transition-colors text-sm"
                                >
                                    Insights
                                </button>

                                <button
                                    onClick={() => scrollToSection('how-it-works')}
                                    className="text-slate-300 hover:text-white transition-colors text-sm"
                                >
                                    Methodology
                                </button>
                            </div>
                        </div>

                        {/* Desktop CTA */}
                        <button
                            onClick={() => scrollToSection('map-preview')}
                            className="hidden md:inline-flex px-6 py-2.5 bg-blue-600 hover:bg-blue-700 text-white rounded-lg transition-all text-sm font-medium"
                        >
                            Explore the Map
                        </button>

                        {/* Mobile Menu Button */}
                        <button
                            className="md:hidden text-slate-300 hover:text-white"
                            onClick={() => setMobileOpen(true)}
                        >
                            <Menu className="w-6 h-6" />
                        </button>
                    </div>
                </div>
            </nav>

            {/* MOBILE MENU */}
            <AnimatePresence>
                {mobileOpen && (
                    <motion.div
                        initial={{ opacity: 0 }}
                        animate={{ opacity: 1 }}
                        exit={{ opacity: 0 }}
                        className="fixed inset-0 z-50 bg-slate-950/95 backdrop-blur-xl"
                    >
                        <div className="flex items-center justify-between p-6">
                            <span className="text-xl font-semibold text-white">FEASIX</span>
                            <button onClick={() => setMobileOpen(false)}>
                                <X className="w-6 h-6 text-slate-300" />
                            </button>
                        </div>

                        <div className="flex flex-col gap-6 px-6 mt-12">
                            {[
                                ['Product', 'product-value'],
                                ['Technology', 'hardware'],
                                ['Methodology', 'how-it-works'],
                                ['Insights', 'map-preview'],
                            ].map(([label, target]) => (
                                <button
                                    key={label}
                                    onClick={() => scrollToSection(target)}
                                    className="text-xl text-slate-300 hover:text-white text-left"
                                >
                                    {label}
                                </button>
                            ))}

                            <button
                                onClick={() => scrollToSection('map-preview')}
                                className="mt-6 px-6 py-3 bg-blue-600 hover:bg-blue-700 text-white rounded-lg text-base font-medium"
                            >
                                Explore the Map
                            </button>
                        </div>
                    </motion.div>
                )}
            </AnimatePresence>
        </>
    );
}
