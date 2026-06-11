import { QueryClient } from "@tanstack/react-query";

export const queryClient = new QueryClient({
  defaultOptions: {
    queries: {
      staleTime: 5 * 60 * 1000,   // 5 min — site data changes infrequently
      gcTime: 10 * 60 * 1000,     // 10 min cache retention
      retry: 1,
      refetchOnWindowFocus: false, // map workspace; focus events should not re-fetch
    },
  },
});
