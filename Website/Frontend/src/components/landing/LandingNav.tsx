'use client';

import { useState, useRef, useEffect } from 'react';
import Link from 'next/link';
import { ChevronDown, Menu, X } from 'lucide-react';
import { AnimatePresence, motion } from 'framer-motion';

interface DropdownItem {
  label: string;
  action: () => void;
}

interface NavItem {
  label: string;
  items: DropdownItem[];
}

function scrollTo(sectionId: string) {
  const el = document.getElementById(sectionId);
  if (el) el.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

export function LandingNav() {
  const [activeDropdown, setActiveDropdown] = useState<string | null>(null);
  const [mobileOpen, setMobileOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  const navItems: NavItem[] = [
    {
      label: 'Product',
      items: [
        { label: 'Feasibility Engine',     action: () => scrollTo('product-value') },
        { label: 'Location Intelligence',  action: () => scrollTo('map-preview') },
        { label: 'Financial Modeling',     action: () => scrollTo('product-preview') },
      ],
    },
    {
      label: 'Technology',
      items: [
        { label: 'Edge Hardware',   action: () => scrollTo('hardware') },
        { label: 'Evidence Pipeline',   action: () => scrollTo('how-it-works') },
        { label: 'Decision Intelligence',  action: () => scrollTo('product-preview') },
      ],
    },
    {
      label: 'Use Cases',
      items: [
        { label: 'Entrepreneurs',  action: () => scrollTo('product-value') },
        { label: 'Investors',      action: () => scrollTo('product-value') },
        { label: 'Urban Planning', action: () => scrollTo('map-preview') },
      ],
    },
  ];

  useEffect(() => {
    function handleClickOutside(e: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(e.target as Node)) {
        setActiveDropdown(null);
      }
    }
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  return (
    <>
      <nav className="fixed top-0 left-0 right-0 z-50 bg-page/80 backdrop-blur-xl border-b border-raised/50">
        <div className="max-w-7xl mx-auto px-6 py-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-12">
              <div className="text-2xl font-semibold tracking-tight text-ink-strong">FEASIX</div>

              {/* Desktop nav */}
              <div ref={dropdownRef} className="hidden md:flex items-center gap-8">
                {navItems.map((item) => (
                  <div key={item.label} className="relative">
                    <button
                      onClick={() => setActiveDropdown(activeDropdown === item.label ? null : item.label)}
                      aria-expanded={activeDropdown === item.label}
                      className="flex items-center gap-1 text-ink-body hover:text-ink-strong transition-colors text-sm"
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
                          className="absolute top-full left-0 mt-2 w-56 bg-surface border border-raised rounded-lg shadow-2xl overflow-hidden"
                        >
                          {item.items.map((d) => (
                            <button
                              key={d.label}
                              onClick={() => { d.action(); setActiveDropdown(null); }}
                              className="w-full text-left px-4 py-3 text-sm text-ink-body hover:bg-raised hover:text-ink-strong transition-colors"
                            >
                              {d.label}
                            </button>
                          ))}
                        </motion.div>
                      )}
                    </AnimatePresence>
                  </div>
                ))}

                <button
                  onClick={() => scrollTo('map-preview')}
                  className="text-ink-body hover:text-ink-strong transition-colors text-sm"
                >
                  Insights
                </button>

                <Link
                  href="/how-it-works"
                  className="text-ink-body hover:text-ink-strong transition-colors text-sm"
                >
                  Methodology
                </Link>
              </div>
            </div>

            {/* Desktop CTA */}
            <Link
              href="/workspace"
              className="hidden md:inline-flex px-6 py-2.5 bg-accent hover:bg-accent-hover text-ink-strong rounded-lg transition-all text-sm font-medium"
            >
              Explore the Map
            </Link>

            {/* Mobile toggle */}
            <button
              className="md:hidden text-ink-body hover:text-ink-strong"
              onClick={() => setMobileOpen(true)}
              aria-label="Open menu"
            >
              <Menu className="w-6 h-6" />
            </button>
          </div>
        </div>
      </nav>

      {/* Mobile menu */}
      <AnimatePresence>
        {mobileOpen && (
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            className="fixed inset-0 z-50 bg-page/95 backdrop-blur-xl"
          >
            <div className="flex items-center justify-between p-6">
              <span className="text-xl font-semibold text-ink-strong">FEASIX</span>
              <button onClick={() => setMobileOpen(false)} aria-label="Close menu">
                <X className="w-6 h-6 text-ink-body" />
              </button>
            </div>
            <div className="flex flex-col gap-6 px-6 mt-12">
              {([
                ['Product',     'product-value'],
                ['Technology',  'hardware'],
                ['How It Works','how-it-works'],
                ['Insights',    'map-preview'],
              ] as [string, string][]).map(([label, target]) => (
                <button
                  key={label}
                  onClick={() => { scrollTo(target); setMobileOpen(false); }}
                  className="text-xl text-ink-body hover:text-ink-strong text-left"
                >
                  {label}
                </button>
              ))}
              <Link
                href="/workspace"
                onClick={() => setMobileOpen(false)}
                className="mt-6 px-6 py-3 bg-accent hover:bg-accent-hover text-ink-strong rounded-lg text-base font-medium text-center"
              >
                Explore the Map
              </Link>
            </div>
          </motion.div>
        )}
      </AnimatePresence>
    </>
  );
}
