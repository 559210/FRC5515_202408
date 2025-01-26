public interface ISingleton
{

}

public class Singleton<T> : ISingleton where T : new() {
	
	protected Singleton() {
		
	}
	
	public static T instance {
		get {
			return Nested.instance;
		}
	}
	
	private class Nested {
		
		static Nested() {
			
		}
		
		internal static readonly T instance = new T();
	}
}
