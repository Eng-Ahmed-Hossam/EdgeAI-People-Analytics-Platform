"use client";

import { useRouter } from "next/navigation";
import { useAuth } from "@/contexts/AuthContext";

function formatDate(iso: string): string {
  try {
    return new Date(iso).toLocaleDateString("en-GB", {
      day: "numeric",
      month: "long",
      year: "numeric",
    });
  } catch {
    return iso;
  }
}

export default function AccountClient() {
  const { user, logout } = useAuth();
  const router = useRouter();

  async function handleLogout() {
    await logout();
    router.push("/");
  }

  if (!user) return null;

  return (
    <div className="h-full bg-page px-5 py-8 overflow-y-auto">
      <div className="max-w-xl mx-auto">
        {/* Header */}
        <div className="mb-6">
          <p className="text-[10px] uppercase tracking-widest text-ink-faint font-mono mb-1">
            Account
          </p>
          <h1 className="font-display text-2xl text-ink-strong">
            Your profile
          </h1>
        </div>

        {/* Profile card */}
        <div className="bg-surface border border-hairline rounded-(--r-lg) divide-y divide-(--border-hair)">
          {/* Email row */}
          <div className="px-5 py-4 flex items-center justify-between gap-4">
            <span className="text-xs text-ink-muted shrink-0">
              Email address
            </span>
            <span className="text-sm text-ink-strong font-body truncate text-right">
              {user.email}
            </span>
          </div>

          {/* Member since */}
          <div className="px-5 py-4 flex items-center justify-between gap-4">
            <span className="text-xs text-ink-muted shrink-0">
              Member since
            </span>
            <span className="text-sm text-ink-body font-mono text-right">
              {formatDate(user.createdAt)}
            </span>
          </div>

          {/* Account status */}
          <div className="px-5 py-4 flex items-center justify-between gap-4">
            <span className="text-xs text-ink-muted shrink-0">
              Account status
            </span>
            <span className="text-sm text-[--signal-good] font-mono text-right">
              Active
            </span>
          </div>

          {/* User ID — for reference / support */}
          <div className="px-5 py-4 flex items-center justify-between gap-4">
            <span className="text-xs text-ink-muted shrink-0">
              Account ID
            </span>
            <span className="text-xs text-ink-faint font-mono text-right truncate max-w-48">
              {user.id}
            </span>
          </div>
        </div>

        {/* Future features note */}
        <p className="mt-4 text-xs text-ink-faint">
          Saved reports, billing, and team access will appear here once the
          account services are connected.
        </p>

        {/* Sign out */}
        <div className="mt-8 pt-6 border-t border-hairline">
          <button
            onClick={handleLogout}
            className="px-4 py-2 rounded-(--r-md) text-sm font-body font-medium text-ink-muted border border-hairline hover:border-rule hover:text-ink-body hover:bg-sunken transition-colors"
          >
            Sign out
          </button>
        </div>
      </div>
    </div>
  );
}
