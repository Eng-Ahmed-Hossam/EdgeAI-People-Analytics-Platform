// app/page.tsx
import { Navigation } from '@/components/Navigation';
import { Hero } from '@/components/Hero';
import { ProductValue } from '@/components/ProductValue';
import { HowItWorks } from '@/components/HowItWorks';
import { HardwareSection } from '@/components/HardwareSection';
import { ProductPreview } from '@/components/ProductPreview';
import { MapPreview } from '@/components/MapPreview';
import { TrustSection } from '@/components/TrustSection';
import { FinalCTA } from '@/components/FinalCTA';
import { Footer } from '@/components/Footer';

export default function HomePage() {
  return (
    <main className="min-h-screen bg-slate-950">
      <Navigation />
      <Hero />
      <ProductValue />
      <HowItWorks />
      <HardwareSection />
      <ProductPreview />
      <MapPreview />
      <TrustSection />
      <FinalCTA />
      <Footer />
    </main>
  );
}
