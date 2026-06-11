"use client";

import { useState, type FormEvent } from "react";
import Link from "next/link";
import { useRouter } from "next/navigation";
import { register } from "@/lib/api";

export default function SignupFormClient() {
  const router = useRouter();

  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [confirm, setConfirm] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [pending, setPending] = useState(false);

  async function handleSubmit(e: FormEvent) {
    e.preventDefault();
    setError(null);

    if (password.length < 8) {
      setError("Password must be at least 8 characters.");
      return;
    }
    if (password !== confirm) {
      setError("Passwords do not match.");
      return;
    }

    setPending(true);
    try {
      await register(email, password);
      router.push("/login?registered=1");
    } catch (err) {
      const msg = err instanceof Error ? err.message : String(err);
      // Surface the 409 conflict as a recognisable message.
      if (msg.toLowerCase().includes("already exists")) {
        setError("This email address is already registered.");
      } else {
        setError("Registration failed. Please try again.");
      }
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
          Create an account
        </h1>
        <p className="text-sm text-ink-muted mt-1">
          Start making evidence-based location decisions
        </p>
      </div>

      {/* Form card */}
      <div className="bg-surface border border-hairline rounded-(--r-lg) p-6">
        <form onSubmit={handleSubmit} noValidate>
          {/* Email */}
          <div className="mb-4">
            <label
              htmlFor="signup-email"
              className="block text-xs font-medium text-ink-muted mb-1.5"
            >
              Email address
            </label>
            <input
              id="signup-email"
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
          <div className="mb-4">
            <label
              htmlFor="signup-password"
              className="block text-xs font-medium text-ink-muted mb-1.5"
            >
              Password
              <span className="ml-1 text-ink-faint font-normal">
                (min. 8 characters)
              </span>
            </label>
            <input
              id="signup-password"
              type="password"
              autoComplete="new-password"
              required
              minLength={8}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="••••••••"
              className="w-full h-9 rounded-(--r-md) bg-sunken border border-hairline px-3 text-sm text-ink-strong placeholder:text-ink-faint font-body focus:outline-none focus:border-accent transition-colors"
            />
          </div>

          {/* Confirm password */}
          <div className="mb-5">
            <label
              htmlFor="signup-confirm"
              className="block text-xs font-medium text-ink-muted mb-1.5"
            >
              Confirm password
            </label>
            <input
              id="signup-confirm"
              type="password"
              autoComplete="new-password"
              required
              value={confirm}
              onChange={(e) => setConfirm(e.target.value)}
              placeholder="••••••••"
              className="w-full h-9 rounded-(--r-md) bg-sunken border border-hairline px-3 text-sm text-ink-strong placeholder:text-ink-faint font-body focus:outline-none focus:border-accent transition-colors"
            />
          </div>

          {/* Error */}
          {error && (
            <p role="alert" className="text-sm text-[--signal-weak] mb-4">
              {error}
            </p>
          )}

          {/* Submit */}
          <button
            type="submit"
            disabled={pending}
            className="w-full h-9 rounded-(--r-md) bg-accent text-ink-strong text-sm font-medium font-body hover:bg-accent-hover transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            {pending ? "Creating account…" : "Create account"}
          </button>
        </form>
      </div>

      {/* Login link */}
      <p className="text-center text-sm text-ink-muted mt-4">
        Already have an account?{" "}
        <Link
          href="/login"
          className="text-accent-muted hover:text-ink-strong transition-colors"
        >
          Sign in
        </Link>
      </p>
    </>
  );
}
