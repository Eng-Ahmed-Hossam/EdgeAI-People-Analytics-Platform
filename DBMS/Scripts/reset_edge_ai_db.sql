-- ==========================================
-- RESET SCRIPT: Edge AI Location Intelligence Database
-- ==========================================
-- ⚠️ WARNING: This will delete all tables, data, and relationships
-- Use carefully when you want a clean fresh start.

-- 1️⃣ Drop the entire 'public' schema (this removes all tables, views, sequences, etc.)
DROP SCHEMA IF EXISTS public CASCADE;

-- 2️⃣ Recreate an empty 'public' schema
CREATE SCHEMA public;

-- 3️⃣ Reassign ownership to the default user (usually 'postgres')
ALTER SCHEMA public OWNER TO postgres;

-- 4️⃣ Optional: show a message in console
DO $$
BEGIN
    RAISE NOTICE '✅ All tables dropped and schema reset successfully.';
END $$;

