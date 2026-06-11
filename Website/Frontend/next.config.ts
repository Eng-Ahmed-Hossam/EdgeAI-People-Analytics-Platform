import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // maplibre-gl accesses browser globals (self, window, WebGL) at module
  // evaluation time. Keeping it server-external prevents webpack from
  // analysing it during SSR/static generation and crashing on Vercel.
  serverExternalPackages: ["maplibre-gl"],
};

export default nextConfig;
