import { LandingNav }      from "@/components/landing/LandingNav";
import { Hero }             from "@/components/landing/Hero";
import { ProductValue }     from "@/components/landing/ProductValue";
import { HowItWorks }       from "@/components/landing/HowItWorks";
import { HardwareSection }  from "@/components/landing/HardwareSection";
import { ProductPreview }   from "@/components/landing/ProductPreview";
import { MapPreview }       from "@/components/landing/MapPreview";
import { TrustSection }     from "@/components/landing/TrustSection";
import { FinalCTA }         from "@/components/landing/FinalCTA";
import { LandingFooter }    from "@/components/landing/LandingFooter";

export default function LandingPage() {
  return (
    <main className="min-h-screen bg-page">
      <LandingNav />
      <Hero />
      <ProductValue />
      <HowItWorks />
      <HardwareSection />
      <ProductPreview />
      <MapPreview />
      <TrustSection />
      <FinalCTA />
      <LandingFooter />
    </main>
  );
}
