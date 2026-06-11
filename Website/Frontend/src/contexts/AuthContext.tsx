"use client";

import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useState,
  type ReactNode,
} from "react";
import {
  getMe,
  login as apiLogin,
  logout as apiLogout,
  type UserResponse,
} from "@/lib/api";

interface AuthContextValue {
  /** The authenticated user, or null if not signed in. */
  user: UserResponse | null;
  /** True while the initial session check (GET /api/auth/me) is in flight. */
  isLoading: boolean;
  /** Sign in — calls POST /api/auth/login, sets user on success. */
  login: (email: string, password: string) => Promise<void>;
  /** Sign out — calls POST /api/auth/logout (clears cookie), sets user to null. */
  logout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextValue | null>(null);

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<UserResponse | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Restore session on page load by validating the cookie.
  useEffect(() => {
    getMe()
      .then(setUser)
      .catch(() => setUser(null))
      .finally(() => setIsLoading(false));
  }, []);

  const login = useCallback(async (email: string, password: string) => {
    const userData = await apiLogin(email, password);
    setUser(userData);
  }, []);

  const logout = useCallback(async () => {
    await apiLogout();
    setUser(null);
  }, []);

  return (
    <AuthContext.Provider value={{ user, isLoading, login, logout }}>
      {children}
    </AuthContext.Provider>
  );
}

/**
 * Returns the auth context. Must be called inside a component tree that has
 * AuthProvider as an ancestor (already provided via Providers.tsx → layout).
 */
export function useAuth(): AuthContextValue {
  const ctx = useContext(AuthContext);
  if (!ctx) throw new Error("useAuth must be used within AuthProvider");
  return ctx;
}
