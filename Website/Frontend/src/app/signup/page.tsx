import { Suspense } from "react";
import SignupFormClient from "./_SignupFormClient";

export default function SignupPage() {
  return (
    <div className="min-h-screen bg-page flex items-center justify-center px-4">
      <div className="w-full max-w-90">
        {/* Suspense required: SignupFormClient uses useSearchParams */}
        <Suspense>
          <SignupFormClient />
        </Suspense>
      </div>
    </div>
  );
}
