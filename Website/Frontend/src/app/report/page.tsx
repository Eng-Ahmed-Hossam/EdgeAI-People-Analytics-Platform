"use client";

import { useState } from "react";
import { ExecutiveReport, type ScenarioMode } from "@/components/report/ExecutiveReport";
import {
  strongScenario,
  moderateScenario,
  weakScenario,
  type ReportScenario,
} from "@/lib/reportScenarios";

const reportScenarios: Record<ScenarioMode, ReportScenario> = {
  strong: strongScenario,
  moderate: moderateScenario,
  weak: weakScenario,
};

export default function ReportPage() {
  const [scenario, setScenario] = useState<ScenarioMode>("strong");
  const { assessment, asset } = reportScenarios[scenario];

  return (
    <ExecutiveReport
      assessment={assessment}
      asset={asset}
      mode={scenario}
      onModeChange={setScenario}
    />
  );
}
