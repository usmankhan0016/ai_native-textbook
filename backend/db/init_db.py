"""Initialize Neon Postgres database schema for Feature 007.

This script:
1. Connects to Neon Postgres using DATABASE_URL from .env
2. Creates tables: sessions, messages, selected_text_metadata
3. Creates indexes for query performance
4. Verifies table creation
"""

import os
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    print("❌ ERROR: DATABASE_URL not found in .env file")
    sys.exit(1)

print("🔧 Initializing Neon Postgres database...")
print(f"📍 Host: {DATABASE_URL.split('@')[1].split('/')[0]}")

try:
    # Try asyncpg first (preferred for FastAPI)
    import asyncio
    import asyncpg

    async def init_database():
        """Create database schema using asyncpg."""
        # Remove channel_binding parameter for asyncpg compatibility
        db_url = DATABASE_URL.replace("&channel_binding=require", "").replace("?channel_binding=require", "")

        print("📡 Connecting to Neon Postgres...")
        conn = await asyncpg.connect(db_url)

        try:
            # Read migration SQL
            migration_path = Path(__file__).parent / "migration.sql"
            with open(migration_path, "r") as f:
                migration_sql = f.read()

            print("🏗️  Creating database schema...")

            # Execute entire migration as a single transaction
            # asyncpg supports multi-statement execution
            try:
                await conn.execute(migration_sql)
                print("✅ Database schema created successfully!")
            except Exception as e:
                print(f"   ❌ Migration failed: {e}")
                raise

            # Verify tables
            print("\n📊 Verifying tables...")
            tables = await conn.fetch("""
                SELECT table_name
                FROM information_schema.tables
                WHERE table_schema = 'public'
                AND table_type = 'BASE TABLE'
                ORDER BY table_name;
            """)

            print(f"✅ Found {len(tables)} tables:")
            for table in tables:
                table_name = table['table_name']
                count = await conn.fetchval(f"SELECT COUNT(*) FROM {table_name}")
                print(f"   - {table_name}: {count} rows")

            # Verify indexes
            indexes = await conn.fetch("""
                SELECT indexname, tablename
                FROM pg_indexes
                WHERE schemaname = 'public'
                AND indexname LIKE 'idx_%'
                ORDER BY tablename, indexname;
            """)

            print(f"\n✅ Found {len(indexes)} indexes:")
            for idx in indexes:
                print(f"   - {idx['indexname']} on {idx['tablename']}")

            print("\n🎉 Database initialization complete!")

        finally:
            await conn.close()
            print("🔌 Database connection closed")

    # Run async initialization
    asyncio.run(init_database())

except ImportError:
    print("❌ asyncpg not installed. Install with: pip install asyncpg")
    print("   Alternatively, run the SQL manually:")
    print(f"   psql '{DATABASE_URL}' < backend/db/migration.sql")
    sys.exit(1)

except Exception as e:
    print(f"❌ ERROR: {type(e).__name__}: {e}")
    sys.exit(1)
