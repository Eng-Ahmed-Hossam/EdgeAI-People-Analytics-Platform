import { IntelligenceEmptyState } from "@/components/primitives/IntelligenceStates";

export default function PricingPage() {
  return (
    <div className="flex min-h-screen items-center justify-center bg-page px-6">
      <IntelligenceEmptyState
        className="w-full max-w-2xl"
        kicker="Commercial model"
        summary="Pricing will be structured around decision-grade assessment reports, comparison briefs, and future operational intelligence subscriptions. The current build focuses on product value and assessment quality."
        title="Pricing model is pending commercial packaging"
      />
    </div>
  );
}
