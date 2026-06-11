# EdgeAI Location Intelligence — Frontend

Next.js 15 application (TypeScript, Tailwind CSS v4). See the [repository root README](../README.md) for full setup instructions.

## Development

```bash
npm install
cp .env.local.example .env.local   # or create frontend/.env.local manually
npm run dev
```

Open [http://localhost:3000](http://localhost:3000).

## Environment

Create `frontend/.env.local`:

```
NEXT_PUBLIC_API_URL=http://localhost:8000
```

## Build

```bash
npm run build
npm run start
```

## Lint

```bash
npm run lint
```
