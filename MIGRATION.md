# Migration Guide: Reorganized Project Structure

## What Changed?

The project structure has been reorganized for better clarity and learning flow:

### Old Structure
```
examples/
 basic/
 gui/
 tutorial/
 tools/
 testing/

projects/
 mujo_manipulation/
 rl_training/
 franka_demos/
 hanoi/
 anissa_niryo/
 juan_manip/
```

### New Structure
```
demos/
 01-fundamentals/
 02-visualization/
 03-robotics-arms/
 04-locomotion/
 05-manipulation/
 06-learning-rl/
 07-tools/
 08-testing/

tests/                    (from examples/testing)
docs/                     (for project documentation)
```

## Mapping Old  New

| Old Path | New Path |
|----------|----------|
| examples/basic/ | demos/01-fundamentals/ |
| examples/tutorial/ | demos/01-fundamentals/ |
| examples/gui/ | demos/02-visualization/ |
| projects/franka_demos/ | demos/03-robotics-arms/ |
| projects/anissa_niryo/ | demos/03-robotics-arms/ |
| examples/tools/ | demos/07-tools/ |
| examples/testing/ | tests/ & demos/08-testing/ |
| projects/mujo_manipulation/ | demos/05-manipulation/ |
| projects/juan_manip/ | demos/05-manipulation/ |
| projects/rl_training/ | demos/06-learning-rl/ |
| projects/hanoi/ | demos/06-learning-rl/ |

## Updated Commands

**Old:**
```bash
python examples/basic/basic_tests.py
pytest examples/basic/
```

**New:**
```bash
python demos/01-fundamentals/basic_tests.py
pytest tests/
```

## Benefits of New Structure

 **Logical Organization** - Categories by topic, not by type
 **Clear Learning Path** - Numbered sequence guides new users
 **Scalability** - Easy to add new examples in each category
 **Better Documentation** - Each category has its own README
 **Professional Layout** - Clearer for presentations and sharing

## Old Folders (Now Available for Reference)

The old examples/ and projects/ folders are still present but will be removed once you verify the migration is complete.

To clean up:
```bash
rm -rf examples/ projects/
```

---

**The new structure is now active. Use the demos/ folder going forward!**
