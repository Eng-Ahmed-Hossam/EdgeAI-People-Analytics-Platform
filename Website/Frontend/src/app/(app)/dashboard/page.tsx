import { ExecutiveDashboardWithData } from "@/components/dashboard/ExecutiveDashboard";
import { getExecutiveDashboard } from "@/lib/api";

export default async function DashboardPage() {
  const dashboardData = await getExecutiveDashboard();

  return <ExecutiveDashboardWithData initialData={dashboardData} />;
}
