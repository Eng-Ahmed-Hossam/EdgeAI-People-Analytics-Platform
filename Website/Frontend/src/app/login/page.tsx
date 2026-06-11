import { Suspense } from "react";
import LoginFormClient from "./_LoginFormClient";

export default function LoginPage() {
  return (
    <div className="min-h-screen bg-page flex items-center justify-center px-4">
      <div className="w-full max-w-90">
        {/* Suspense required: LoginFormClient uses useSearchParams */}
        <Suspense>
          <LoginFormClient />
        </Suspense>
      </div>
    </div>
  );
}
