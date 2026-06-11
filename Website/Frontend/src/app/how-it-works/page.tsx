import { IntelligenceEmptyState } from "@/components/primitives/IntelligenceStates";

export default function HowItWorksPage() {
  return (
    <div className="flex min-h-screen items-center justify-center bg-page px-6">
      <IntelligenceEmptyState
        className="w-full max-w-2xl"
        kicker="Methodology brief"
        summary="This surface will explain score construction, confidence intervals, source coverage, and model limitations. The same trust framework is already visible inside reports, comparisons, and the workspace decision brief."
        title="Methodology documentation is staged for final publication"
      />
    </div>
  );
}
