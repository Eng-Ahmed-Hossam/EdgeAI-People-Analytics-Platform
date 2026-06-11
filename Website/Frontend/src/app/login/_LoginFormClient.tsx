"use client";

import { useState, type FormEvent } from "react";
import Link from "next/link";
import { useRouter, useSearchParams } from "next/navigation";
import { useAuth } from "@/contexts/AuthContext";

export default function LoginFormClient() {
  const { login } = useAuth();
  const router = useRouter();
  const params = useSearchParams();
  const next = params.get("next") ?? "/dashboard";

  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [pending, setPending] = useState(false);

  async function handleSubmit(e: FormEvent) {
    e.preventDefault();
    setError(null);
    setPending(true);
    try {
      await login(email, password);
      router.push(next);
    } catch {
      // Generic message regardless of which field was wrong — no user enumeration.
      setError("Invalid email or password.");
    } finally {
      setPending(false);
    }
  }

  return (
    <>
      {/* Brand mark */}
      <div className="mb-8 text-center">
        <Link
          href="/"
          className="inline-flex items-center gap-2 group"
          aria-label="EdgeAI Location Intelligence — home"
        >
          <div
            className="w-6 h-6 rounded-(--r-sm) bg-accent flex items-center justify-center"
            aria-hidden="true"
          >
            <svg
              width="12"
              height="12"
              viewBox="0 0 10 10"
              fill="none"
              stroke="white"
              strokeWidth="1.5"
              strokeLinecap="round"
            >
              <circle cx="5" cy="4" r="2" />
              <path d="M1 9c0-2.2 1.8-4 4-4s4 1.8 4 4" />
            </svg>
          </div>
          <span className="font-display text-base font-semibold text-ink-strong tracking-tight">
            EdgeAI
          </span>
        </Link>
        <h1 className="font-display text-2xl text-ink-strong mt-3 leading-tight">
          Sign in
        </h1>
        <p className="text-sm text-ink-muted mt-1">
          Intelligence for location decisions
        </p>
      </div>

      {/* Form card */}
      <div className="bg-surface border border-hairline rounded-(--r-lg) p-6">
        <form onSubmit={handleSubmit} noValidate>
          {/* Email */}
          <div className="mb-4">
            <label
              htmlFor="login-email"
              className="block text-xs font-medium text-ink-muted mb-1.5"
            >
              Email address
            </label>
            <input
              id="login-email"
              type="email"
              autoComplete="email"
              required
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder="you@example.com"
              className="w-full h-9 rounded-(--r-md) bg-sunken border border-hairline px-3 text-sm text-ink-strong placeholder:text-ink-faint font-body focus:outline-none focus:border-accent transition-colors"
            />
          </div>

          {/* Password */}
          <div className="mb-5">
            <label
              htmlFor="login-password"
              className="block text-xs font-medium text-ink-muted mb-1.5"
            >
              Password
            </label>
            <input
              id="login-password"
              type="password"
              autoComplete="current-password"
              required
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="••••••••"
              className="w-full h-9 rounded-(--r-md) bg-sunken border border-hairline px-3 text-sm text-ink-strong placeholder:text-ink-faint font-body focus:outline-none focus:border-accent transition-colors"
            />
          </div>

          {/* Error */}
          {error && (
            <p
              role="alert"
              className="text-sm text-[--signal-weak] mb-4"
            >
              {error}
            </p>
          )}

          {/* Submit */}
          <button
            type="submit"
            disabled={pending}
            className="w-full h-9 rounded-(--r-md) bg-accent text-ink-strong text-sm font-medium font-body hover:bg-accent-hover transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            {pending ? "Signing in…" : "Sign in"}
          </button>
        </form>
      </div>

      {/* Register link */}
      <p className="text-center text-sm text-ink-muted mt-4">
        No account?{" "}
        <Link
          href="/signup"
          className="text-accent-muted hover:text-ink-strong transition-colors"
        >
          Create one
        </Link>
      </p>
    </>
  );
}
