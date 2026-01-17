import Link from 'next/link';
import { Twitter, Linkedin, Github } from 'lucide-react';

export function Footer() {
    return (
        <footer className="bg-slate-950 border-t border-slate-900">
            <div className="max-w-7xl mx-auto px-6 py-16">
                <div className="grid grid-cols-1 md:grid-cols-4 gap-12 mb-12">
                    {/* Brand */}
                    <div>
                        <div className="text-2xl font-semibold text-white mb-4">FEASIX</div>
                        <p className="text-slate-400 text-sm leading-relaxed">
                            Professional business feasibility intelligence powered by real-world
                            location activity and financial modeling.
                        </p>
                    </div>

                    {/* Product */}
                    <div>
                        <h4 className="text-white font-medium mb-4">Product</h4>
                        <ul className="space-y-3">
                            <li>
                                <a
                                    href="#how-it-works"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Feasibility Engine
                                </a>
                            </li>
                            <li>
                                <a
                                    href="#map-preview"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Location Intelligence
                                </a>
                            </li>
                            <li>
                                <a
                                    href="#product-preview"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Financial Modeling
                                </a>
                            </li>
                            <li>
                                <Link
                                    href="/pricing"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Pricing
                                </Link>
                            </li>
                        </ul>
                    </div>

                    {/* Company */}
                    <div>
                        <h4 className="text-white font-medium mb-4">Company</h4>
                        <ul className="space-y-3">
                            <li>
                                <Link
                                    href="/about"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    About Us
                                </Link>
                            </li>
                            <li>
                                <Link
                                    href="/methodology"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Methodology
                                </Link>
                            </li>
                            <li>
                                <Link
                                    href="/careers"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Careers
                                </Link>
                            </li>
                            <li>
                                <Link
                                    href="/contact"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Contact
                                </Link>
                            </li>
                        </ul>
                    </div>

                    {/* Resources */}
                    <div>
                        <h4 className="text-white font-medium mb-4">Resources</h4>
                        <ul className="space-y-3">
                            <li>
                                <Link
                                    href="/docs"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Documentation
                                </Link>
                            </li>
                            <li>
                                <Link
                                    href="/api"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    API Reference
                                </Link>
                            </li>
                            <li>
                                <Link
                                    href="/privacy"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Privacy Policy
                                </Link>
                            </li>
                            <li>
                                <Link
                                    href="/terms"
                                    className="text-slate-400 hover:text-white text-sm transition-colors"
                                >
                                    Terms of Service
                                </Link>
                            </li>
                        </ul>
                    </div>
                </div>

                {/* Bottom Bar */}
                <div className="pt-8 border-t border-slate-900 flex flex-col md:flex-row items-center justify-between gap-4">
                    <p className="text-sm text-slate-500">
                        © 2026 FEASIX. All rights reserved.
                    </p>

                    <div className="flex items-center gap-6">
                        <a
                            href="https://twitter.com"
                            target="_blank"
                            rel="noopener noreferrer"
                            aria-label="Twitter"
                            className="text-slate-500 hover:text-white transition-colors"
                        >
                            <Twitter className="w-5 h-5" />
                        </a>
                        <a
                            href="https://linkedin.com"
                            target="_blank"
                            rel="noopener noreferrer"
                            aria-label="LinkedIn"
                            className="text-slate-500 hover:text-white transition-colors"
                        >
                            <Linkedin className="w-5 h-5" />
                        </a>
                        <a
                            href="https://github.com"
                            target="_blank"
                            rel="noopener noreferrer"
                            aria-label="GitHub"
                            className="text-slate-500 hover:text-white transition-colors"
                        >
                            <Github className="w-5 h-5" />
                        </a>
                    </div>
                </div>
            </div>
        </footer>
    );
}
